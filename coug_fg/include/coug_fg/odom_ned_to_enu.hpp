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

#pragma once

#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include "coug_fg/odom_ned_to_enu_parameters.hpp"

namespace coug_fg {

class OdomNedToEnuNode : public rclcpp::Node {
 public:
  explicit OdomNedToEnuNode(const rclcpp::NodeOptions& options);

 private:
  // --- Callbacks ---
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr& msg);

  // --- Helpers ---
  static nav_msgs::msg::Odometry convertToEnu(const nav_msgs::msg::Odometry::SharedPtr& msg);

  // --- ROS Interfaces ---
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  // --- Parameters ---
  std::shared_ptr<odom_ned_to_enu_node::ParamListener> param_listener_;
  odom_ned_to_enu_node::Params params_;
};

}  // namespace coug_fg
