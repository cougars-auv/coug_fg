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

#include <GeographicLib/LocalCartesian.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <string>

#include "coug_fg/navsat_odom_parameters.hpp"

namespace coug_fg {

class NavsatOdomNode : public rclcpp::Node {
 public:
  explicit NavsatOdomNode(const rclcpp::NodeOptions& options);

 private:
  // --- Callbacks ---
  void originCallback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr& msg);

  void navsatCallback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr& msg);

  // --- Helpers ---
  void setOrigin(const sensor_msgs::msg::NavSatFix& msg);

  nav_msgs::msg::Odometry convertToOdom(const sensor_msgs::msg::NavSatFix::ConstSharedPtr& msg);

  // --- Diagnostics ---
  void checkOriginStatus(diagnostic_updater::DiagnosticStatusWrapper& stat);

  // --- ROS Interfaces ---
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr navsat_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr origin_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr origin_pub_;
  rclcpp::TimerBase::SharedPtr origin_timer_;
  diagnostic_updater::Updater diagnostic_updater_;

  // --- Parameters ---
  std::shared_ptr<navsat_odom_node::ParamListener> param_listener_;
  navsat_odom_node::Params params_;

  // --- State ---
  GeographicLib::LocalCartesian local_cartesian_;
  sensor_msgs::msg::NavSatFix origin_navsat_;
  bool origin_set_{false};
};

}  // namespace coug_fg
