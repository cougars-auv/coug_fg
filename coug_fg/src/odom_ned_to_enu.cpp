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

#include "coug_fg/odom_ned_to_enu.hpp"

#include <cmath>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "coug_fg/odom_ned_to_enu_parameters.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace coug_fg {

OdomNedToEnuNode::OdomNedToEnuNode(const rclcpp::NodeOptions& options)
    : Node("odom_ned_to_enu_node", options) {
  param_listener_ =
      std::make_shared<odom_ned_to_enu_node::ParamListener>(get_node_parameters_interface());
  params_ = param_listener_->get_params();

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      params_.input_topic, rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::ConstSharedPtr& msg) { odomCallback(msg); });

  odom_pub_ =
      create_publisher<nav_msgs::msg::Odometry>(params_.output_topic, rclcpp::SystemDefaultsQoS());

  RCLCPP_INFO(get_logger(), "Initialization complete.");
}

void OdomNedToEnuNode::odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& msg) {
  odom_pub_->publish(convertToEnu(msg));
}

auto OdomNedToEnuNode::convertToEnu(const nav_msgs::msg::Odometry::ConstSharedPtr& msg)
    -> nav_msgs::msg::Odometry {
  nav_msgs::msg::Odometry odom_msg = *msg;

  // Convert NED -> ENU
  static const geometry_msgs::msg::TransformStamped kNedToEnu = []() {
    geometry_msgs::msg::TransformStamped transform;
    transform.transform.rotation = tf2::toMsg(tf2::Quaternion(M_SQRT1_2, M_SQRT1_2, 0.0, 0.0));
    return transform;
  }();

  if (msg->pose.covariance[0] >= 0.0) {
    // Pose orientation covariance is expressed about the world-frame axes
    tf2::doTransform(msg->pose, odom_msg.pose, kNedToEnu);
  } else {
    tf2::doTransform(msg->pose.pose, odom_msg.pose.pose, kNedToEnu);
  }

  return odom_msg;
}

}  // namespace coug_fg

RCLCPP_COMPONENTS_REGISTER_NODE(coug_fg::OdomNedToEnuNode)
