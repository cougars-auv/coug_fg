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

#pragma once

#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include "coug_fg/mavros_odom_covariance_parameters.hpp"

namespace coug_fg {

class MavrosOdomCovarianceNode : public rclcpp::Node {
 public:
  explicit MavrosOdomCovarianceNode(const rclcpp::NodeOptions& options);

 private:
  // --- Callbacks ---
  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& msg);

  // --- Helpers ---
  auto convertToOdom(const nav_msgs::msg::Odometry::ConstSharedPtr& msg) const
      -> nav_msgs::msg::Odometry;

  // --- ROS Interfaces ---
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  // --- Parameters ---
  std::shared_ptr<mavros_odom_covariance_node::ParamListener> param_listener_;
  mavros_odom_covariance_node::Params params_;
};

}  // namespace coug_fg
