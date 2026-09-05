// Copyright (c) 2026 BYU FROST Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "coug_fg/dvl_a50_odom.hpp"

#include <rcl/time.h>

#include <cmath>
#include <cstdint>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/convert.hpp>
#include <tf2/exceptions.hpp>
#include <tf2/time.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include "coug_fg/dvl_a50_odom_parameters.hpp"
#include "dvl_msgs/msg/dvldr.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace coug_fg {

DvlA50OdomNode::DvlA50OdomNode(const rclcpp::NodeOptions& options)
    : Node("dvl_a50_odom_node", options) {
  param_listener_ =
      std::make_shared<dvl_a50_odom_node::ParamListener>(get_node_parameters_interface());
  params_ = param_listener_->get_params();

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  dvl_sub_ = create_subscription<dvl_msgs::msg::DVLDR>(
      params_.input_topic, rclcpp::SensorDataQoS(),
      [this](const dvl_msgs::msg::DVLDR::ConstSharedPtr& msg) { dvlCallback(msg); });

  odom_pub_ =
      create_publisher<nav_msgs::msg::Odometry>(params_.output_topic, rclcpp::SystemDefaultsQoS());

  RCLCPP_INFO(get_logger(), "Initialization complete.");
}

void DvlA50OdomNode::dvlCallback(const dvl_msgs::msg::DVLDR::ConstSharedPtr& msg) {
  const std::string dvl_frame =
      params_.use_parameter_frame ? params_.parameter_frame : msg->header.frame_id;

  geometry_msgs::msg::TransformStamped dvl_T_base_tf;
  try {
    dvl_T_base_tf = tf_buffer_->lookupTransform(dvl_frame, params_.base_frame, tf2::TimePointZero);
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Could not transform %s to %s: %s",
                         dvl_frame.c_str(), params_.base_frame.c_str(), ex.what());
    return;
  }

  odom_pub_->publish(convertToOdom(msg, dvl_frame, dvl_T_base_tf));
}

nav_msgs::msg::Odometry DvlA50OdomNode::convertToOdom(
    const dvl_msgs::msg::DVLDR::ConstSharedPtr& msg, const std::string& dvl_frame,
    const geometry_msgs::msg::TransformStamped& dvl_T_base_tf) const {
  // Transform the DVL pose to the base pose, both in the odom frame
  geometry_msgs::msg::Pose dvl_T_base;
  dvl_T_base.position.x = dvl_T_base_tf.transform.translation.x;
  dvl_T_base.position.y = dvl_T_base_tf.transform.translation.y;
  dvl_T_base.position.z = dvl_T_base_tf.transform.translation.z;
  dvl_T_base.orientation = dvl_T_base_tf.transform.rotation;

  geometry_msgs::msg::TransformStamped odom_T_dvl_tf;
  odom_T_dvl_tf.header.frame_id = params_.odom_frame;
  odom_T_dvl_tf.child_frame_id = dvl_frame;
  odom_T_dvl_tf.transform.translation.x = msg->position.x;
  odom_T_dvl_tf.transform.translation.y = msg->position.y;
  odom_T_dvl_tf.transform.translation.z = msg->position.z;

  static constexpr double kDegToRad = M_PI / 180.0;
  tf2::Quaternion q;
  q.setRPY(msg->roll * kDegToRad, msg->pitch * kDegToRad, msg->yaw * kDegToRad);

  // Convert FRD -> FLU
  static const tf2::Quaternion kFrdToFlu(1.0, 0.0, 0.0, 0.0);
  q *= kFrdToFlu;

  odom_T_dvl_tf.transform.rotation = tf2::toMsg(q);

  geometry_msgs::msg::Pose odom_T_base;
  tf2::doTransform(dvl_T_base, odom_T_base, odom_T_dvl_tf);

  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.frame_id = params_.odom_frame;

  odom_msg.child_frame_id = params_.base_frame;

  if (params_.override_timestamp) {
    odom_msg.header.stamp = msg->header.stamp;
  } else {
    static constexpr double kSecondsToNanoseconds = 1e9;
    const double whole_sec = std::floor(msg->time);
    const auto sec = static_cast<int32_t>(whole_sec);
    const auto nanosec = static_cast<uint32_t>((msg->time - whole_sec) * kSecondsToNanoseconds);
    odom_msg.header.stamp = rclcpp::Time(sec, nanosec, RCL_ROS_TIME);
  }

  odom_msg.pose.pose = odom_T_base;

  const double var_pos = msg->pos_std * msg->pos_std;
  odom_msg.pose.covariance[0] = var_pos;
  odom_msg.pose.covariance[7] = var_pos;
  odom_msg.pose.covariance[14] = var_pos;

  const auto& sigmas = params_.orientation_noise_sigmas;
  odom_msg.pose.covariance[21] = sigmas[0] * sigmas[0];
  odom_msg.pose.covariance[28] = sigmas[1] * sigmas[1];
  odom_msg.pose.covariance[35] = sigmas[2] * sigmas[2];

  return odom_msg;
}

}  // namespace coug_fg

RCLCPP_COMPONENTS_REGISTER_NODE(coug_fg::DvlA50OdomNode)
