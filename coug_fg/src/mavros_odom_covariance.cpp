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

#include "coug_fg/mavros_odom_covariance.hpp"

#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "coug_fg/mavros_odom_covariance_parameters.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace coug_fg {

MavrosOdomCovarianceNode::MavrosOdomCovarianceNode(const rclcpp::NodeOptions& options)
    : Node("mavros_odom_covariance_node", options) {
  param_listener_ =
      std::make_shared<mavros_odom_covariance_node::ParamListener>(get_node_parameters_interface());
  params_ = param_listener_->get_params();

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      params_.input_topic, rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::ConstSharedPtr& msg) { odomCallback(msg); });

  odom_pub_ =
      create_publisher<nav_msgs::msg::Odometry>(params_.output_topic, rclcpp::SystemDefaultsQoS());

  RCLCPP_INFO(get_logger(), "Initialization complete.");
}

void MavrosOdomCovarianceNode::odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& msg) {
  odom_pub_->publish(convertToOdom(msg));
}

auto MavrosOdomCovarianceNode::convertToOdom(
    const nav_msgs::msg::Odometry::ConstSharedPtr& msg) const -> nav_msgs::msg::Odometry {
  nav_msgs::msg::Odometry odom_msg = *msg;

  const auto& position_sigmas = params_.position_noise_sigmas;
  odom_msg.pose.covariance[0] = position_sigmas[0] * position_sigmas[0];
  odom_msg.pose.covariance[7] = position_sigmas[1] * position_sigmas[1];
  odom_msg.pose.covariance[14] = position_sigmas[2] * position_sigmas[2];

  const auto& orientation_sigmas = params_.orientation_noise_sigmas;
  odom_msg.pose.covariance[21] = orientation_sigmas[0] * orientation_sigmas[0];
  odom_msg.pose.covariance[28] = orientation_sigmas[1] * orientation_sigmas[1];
  odom_msg.pose.covariance[35] = orientation_sigmas[2] * orientation_sigmas[2];

  const auto& vel_sigmas = params_.velocity_noise_sigmas;
  odom_msg.twist.covariance[0] = vel_sigmas[0] * vel_sigmas[0];
  odom_msg.twist.covariance[7] = vel_sigmas[1] * vel_sigmas[1];
  odom_msg.twist.covariance[14] = vel_sigmas[2] * vel_sigmas[2];

  const auto& ang_vel_sigmas = params_.angular_velocity_noise_sigmas;
  odom_msg.twist.covariance[21] = ang_vel_sigmas[0] * ang_vel_sigmas[0];
  odom_msg.twist.covariance[28] = ang_vel_sigmas[1] * ang_vel_sigmas[1];
  odom_msg.twist.covariance[35] = ang_vel_sigmas[2] * ang_vel_sigmas[2];

  return odom_msg;
}

}  // namespace coug_fg

RCLCPP_COMPONENTS_REGISTER_NODE(coug_fg::MavrosOdomCovarianceNode)
