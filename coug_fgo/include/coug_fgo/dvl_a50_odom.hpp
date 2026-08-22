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

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <dvl_msgs/msg/dvldr.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "coug_fgo/dvl_a50_odom_parameters.hpp"

namespace coug_fgo {

class DvlA50OdomNode : public rclcpp::Node {
 public:
  explicit DvlA50OdomNode(const rclcpp::NodeOptions& options);

 private:
  // --- Callbacks ---
  void dvlCallback(const dvl_msgs::msg::DVLDR::SharedPtr msg);

  // --- Helpers ---
  nav_msgs::msg::Odometry convertToOdom(const dvl_msgs::msg::DVLDR::SharedPtr msg,
                                        const std::string& dvl_frame,
                                        const geometry_msgs::msg::TransformStamped& dvl_T_base_tf);

  // --- ROS Interfaces ---
  rclcpp::Subscription<dvl_msgs::msg::DVLDR>::SharedPtr dvl_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // --- Parameters ---
  std::shared_ptr<dvl_a50_odom_node::ParamListener> param_listener_;
  dvl_a50_odom_node::Params params_;
};

}  // namespace coug_fgo
