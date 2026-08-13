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
 * @file seatrac_x150_imu_depth.hpp
 * @brief ROS 2 node that converts SeaTrac ModemStatus to IMU, magnetometer, and depth messages.
 * @author Nelson Durrant
 * @date May 2026
 */

#pragma once

#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <seatrac_interfaces/msg/modem_status.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>

#include "coug_fgo/seatrac_x150_imu_depth_parameters.hpp"

namespace coug_fgo {

/**
 * @class SeatracX150ImuDepthNode
 * @brief ROS 2 node that converts SeaTrac ModemStatus to IMU, magnetometer, and depth messages.
 */
class SeatracX150ImuDepthNode : public rclcpp::Node {
 public:
  /**
   * @brief Constructs the node and sets up the ModemStatus conversion.
   * @param options The node options.
   */
  explicit SeatracX150ImuDepthNode(const rclcpp::NodeOptions& options);

 private:
  /**
   * @brief Publishes IMU, magnetometer, and/or depth messages when the modem report includes them.
   * @param msg The incoming ModemStatus message.
   */
  void modemStatusCallback(const seatrac_interfaces::msg::ModemStatus::SharedPtr msg);

  /**
   * @brief Builds a NED IMU message from the modem attitude with declination correction.
   * @param msg The incoming ModemStatus message (attitude in 0.1-degree units).
   * @return The converted Imu message; rates and accelerations are flagged unmeasured.
   */
  sensor_msgs::msg::Imu convertToImu(const seatrac_interfaces::msg::ModemStatus::SharedPtr msg);

  /**
   * @brief Builds a MagneticField message from the modem field readings.
   * @param msg The incoming ModemStatus message.
   * @return The converted MagneticField message; the covariance comes from parameter sigmas.
   */
  sensor_msgs::msg::MagneticField convertToMag(
      const seatrac_interfaces::msg::ModemStatus::SharedPtr msg);

  /**
   * @brief Builds a NED depth Odometry message from the modem pressure sensor reading.
   * @param msg The incoming ModemStatus message (depth in 0.1-meter units, positive down).
   * @return The converted Odometry message; only the Z position and its variance are populated.
   */
  nav_msgs::msg::Odometry convertToOdom(const seatrac_interfaces::msg::ModemStatus::SharedPtr msg);

  // --- ROS Interfaces ---
  rclcpp::Subscription<seatrac_interfaces::msg::ModemStatus>::SharedPtr modem_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr depth_pub_;

  // --- Parameters ---
  std::shared_ptr<seatrac_x150_imu_depth_node::ParamListener> param_listener_;
  seatrac_x150_imu_depth_node::Params params_;
};

}  // namespace coug_fgo
