// Copyright 2026 BYU FROST Lab
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

#include "coug_fg/odom_child_to_base.hpp"

#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2/exceptions.hpp>
#include <tf2/time.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include "coug_fg/odom_child_to_base_parameters.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace coug_fg {

OdomChildToBaseNode::OdomChildToBaseNode(const rclcpp::NodeOptions& options)
    : Node("odom_child_to_base_node", options) {
  param_listener_ =
      std::make_shared<odom_child_to_base_node::ParamListener>(get_node_parameters_interface());
  params_ = param_listener_->get_params();

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      params_.input_topic, rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::ConstSharedPtr& msg) { odomCallback(msg); });

  odom_pub_ =
      create_publisher<nav_msgs::msg::Odometry>(params_.output_topic, rclcpp::SystemDefaultsQoS());

  RCLCPP_INFO(get_logger(), "Initialization complete.");
}

void OdomChildToBaseNode::odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& msg) {
  const auto& parent_R_child = msg->pose.pose.orientation;
  if (parent_R_child.x == 0.0 && parent_R_child.y == 0.0 && parent_R_child.z == 0.0) {
    RCLCPP_WARN(get_logger(), "Rejected odometry: '%s' orientation is unset.",
                msg->child_frame_id.c_str());
    return;
  }

  geometry_msgs::msg::TransformStamped child_T_base_tf;
  try {
    child_T_base_tf =
        tf_buffer_->lookupTransform(msg->child_frame_id, params_.base_frame, tf2::TimePointZero);
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "Failed to look up transform from '%s' to '%s': %s",
                         params_.base_frame.c_str(), msg->child_frame_id.c_str(), ex.what());
    return;
  }

  odom_pub_->publish(convertToOdom(msg, child_T_base_tf));
}

auto OdomChildToBaseNode::convertToOdom(
    const nav_msgs::msg::Odometry::ConstSharedPtr& msg,
    const geometry_msgs::msg::TransformStamped& child_T_base_tf) const -> nav_msgs::msg::Odometry {
  nav_msgs::msg::Odometry odom_msg = *msg;
  odom_msg.child_frame_id = params_.base_frame;

  // Transform the child pose to the base pose, both in the parent frame
  tf2::Transform parent_T_child;
  tf2::Transform child_T_base;
  tf2::fromMsg(msg->pose.pose, parent_T_child);
  tf2::fromMsg(child_T_base_tf.transform, child_T_base);
  tf2::toMsg(parent_T_child * child_T_base, odom_msg.pose.pose);

  return odom_msg;
}

}  // namespace coug_fg

RCLCPP_COMPONENTS_REGISTER_NODE(coug_fg::OdomChildToBaseNode)
