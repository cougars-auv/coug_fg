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
#include <seatrac_interfaces/msg/modem_status.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "coug_fg/seatrac_x150_imu_depth_parameters.hpp"

namespace coug_fg {

class SeatracX150ImuDepthNode : public rclcpp::Node {
 public:
  explicit SeatracX150ImuDepthNode(const rclcpp::NodeOptions& options);

 private:
  // --- Callbacks ---
  void modemStatusCallback(const seatrac_interfaces::msg::ModemStatus::ConstSharedPtr& msg);

  // --- Helpers ---
  auto convertToImu(const seatrac_interfaces::msg::ModemStatus::ConstSharedPtr& msg) const
      -> sensor_msgs::msg::Imu;

  auto convertToOdom(const seatrac_interfaces::msg::ModemStatus::ConstSharedPtr& msg) const
      -> nav_msgs::msg::Odometry;

  // --- ROS Interfaces ---
  rclcpp::Subscription<seatrac_interfaces::msg::ModemStatus>::SharedPtr modem_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr depth_pub_;

  // --- Parameters ---
  std::shared_ptr<seatrac_x150_imu_depth_node::ParamListener> param_listener_;
  seatrac_x150_imu_depth_node::Params params_;
};

}  // namespace coug_fg
