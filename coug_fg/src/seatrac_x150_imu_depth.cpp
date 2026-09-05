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

#include "coug_fg/seatrac_x150_imu_depth.hpp"

#include <cmath>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "coug_fg/seatrac_x150_imu_depth_parameters.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "seatrac_interfaces/msg/modem_status.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace coug_fg {

SeatracX150ImuDepthNode::SeatracX150ImuDepthNode(const rclcpp::NodeOptions& options)
    : Node("seatrac_x150_imu_depth_node", options) {
  param_listener_ =
      std::make_shared<seatrac_x150_imu_depth_node::ParamListener>(get_node_parameters_interface());
  params_ = param_listener_->get_params();

  modem_sub_ = create_subscription<seatrac_interfaces::msg::ModemStatus>(
      params_.input_topic, rclcpp::SensorDataQoS(),
      [this](const seatrac_interfaces::msg::ModemStatus::ConstSharedPtr& msg) {
        modemStatusCallback(msg);
      });

  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(params_.imu_output_topic,
                                                     rclcpp::SystemDefaultsQoS());

  depth_pub_ = create_publisher<nav_msgs::msg::Odometry>(params_.depth_output_topic,
                                                         rclcpp::SystemDefaultsQoS());

  RCLCPP_INFO(get_logger(), "Initialization complete.");
}

void SeatracX150ImuDepthNode::modemStatusCallback(
    const seatrac_interfaces::msg::ModemStatus::ConstSharedPtr& msg) {
  if (msg->includes_local_attitude) {
    imu_pub_->publish(convertToImu(msg));
  }

  if (msg->includes_env_fields) {
    depth_pub_->publish(convertToOdom(msg));
  }
}

auto SeatracX150ImuDepthNode::convertToImu(
    const seatrac_interfaces::msg::ModemStatus::ConstSharedPtr& msg) const
    -> sensor_msgs::msg::Imu {
  sensor_msgs::msg::Imu imu_msg;
  imu_msg.header = msg->header;
  if (params_.use_parameter_frame) {
    imu_msg.header.frame_id = params_.parameter_frame;
  }

  static constexpr double kSeatracToRad = M_PI / 1800.0;
  const double roll_rad = msg->attitude_roll * kSeatracToRad;
  const double pitch_rad = msg->attitude_pitch * kSeatracToRad;
  const double yaw_rad = msg->attitude_yaw * kSeatracToRad + params_.mag_declination_radians;

  tf2::Quaternion q;
  q.setRPY(roll_rad, pitch_rad, yaw_rad);

  // Convert FRD -> FLU
  static const tf2::Quaternion kFrdToFlu(1.0, 0.0, 0.0, 0.0);
  q *= kFrdToFlu;

  imu_msg.orientation = tf2::toMsg(q);

  const auto& sigmas = params_.orientation_noise_sigmas;
  imu_msg.orientation_covariance[0] = sigmas[0] * sigmas[0];
  imu_msg.orientation_covariance[4] = sigmas[1] * sigmas[1];
  imu_msg.orientation_covariance[8] = sigmas[2] * sigmas[2];

  static constexpr double kUnknownCovariance = -1.0;
  imu_msg.linear_acceleration_covariance[0] = kUnknownCovariance;
  imu_msg.angular_velocity_covariance[0] = kUnknownCovariance;

  return imu_msg;
}

auto SeatracX150ImuDepthNode::convertToOdom(
    const seatrac_interfaces::msg::ModemStatus::ConstSharedPtr& msg) const
    -> nav_msgs::msg::Odometry {
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = msg->header.stamp;
  odom_msg.header.frame_id = params_.map_frame;

  odom_msg.child_frame_id =
      params_.use_parameter_frame ? params_.parameter_frame : msg->header.frame_id;

  static constexpr double kSeatracToMeters = 0.1;
  odom_msg.pose.pose.position.z = msg->depth_local * kSeatracToMeters;
  odom_msg.pose.pose.orientation.w = 1.0;

  const double var_depth = params_.depth_noise_sigma * params_.depth_noise_sigma;
  odom_msg.pose.covariance[14] = var_depth;

  return odom_msg;
}

}  // namespace coug_fg

RCLCPP_COMPONENTS_REGISTER_NODE(coug_fg::SeatracX150ImuDepthNode)
