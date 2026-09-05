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
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>

#include "coug_fg/sbg_imu_mag_parameters.hpp"

namespace coug_fg {

class SbgImuMagNode : public rclcpp::Node {
 public:
  explicit SbgImuMagNode(const rclcpp::NodeOptions& options);

 private:
  // --- Callbacks ---
  void magCallback(const sensor_msgs::msg::MagneticField::ConstSharedPtr& msg);

  // --- Helpers ---
  auto convertToTesla(const sensor_msgs::msg::MagneticField::ConstSharedPtr& msg) const
      -> sensor_msgs::msg::MagneticField;

  // --- ROS Interfaces ---
  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_sub_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;

  // --- Parameters ---
  std::shared_ptr<sbg_imu_mag_node::ParamListener> param_listener_;
  sbg_imu_mag_node::Params params_;
};

}  // namespace coug_fg
