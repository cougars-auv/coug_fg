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

/**
 * @file sbg_imu_mag.hpp
 * @brief ROS 2 node that converts SBG magnetometer output from arbitrary units to nominal Tesla.
 * @author Nelson Durrant
 * @date August 2026
 */

#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>

#include "coug_fgo/sbg_imu_mag_parameters.hpp"

namespace coug_fgo {

/**
 * @class SbgImuMagNode
 * @brief ROS 2 node that converts SBG magnetometer output from arbitrary units to nominal Tesla.
 */
class SbgImuMagNode : public rclcpp::Node {
 public:
  /**
   * @brief Constructs the node and sets up the arbitrary units to nominal Tesla conversion.
   * @param options The node options.
   */
  explicit SbgImuMagNode(const rclcpp::NodeOptions& options);

 private:
  /**
   * @brief Publishes the Tesla-scaled copy of an incoming magnetometer message.
   * @param msg The incoming magnetometer message, in arbitrary units.
   */
  void magCallback(const sensor_msgs::msg::MagneticField::SharedPtr msg);

  /**
   * @brief Scales the field and its covariance from arbitrary units into nominal Tesla.
   * @param msg The incoming magnetometer message, in arbitrary units.
   * @return The converted magnetometer message.
   */
  sensor_msgs::msg::MagneticField convertToTesla(
      const sensor_msgs::msg::MagneticField::SharedPtr msg);

  // --- ROS Interfaces ---
  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_sub_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;

  // --- Parameters ---
  std::shared_ptr<sbg_imu_mag_node::ParamListener> param_listener_;
  sbg_imu_mag_node::Params params_;
};

}  // namespace coug_fgo
