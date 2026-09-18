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

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/quaternion.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>

#include "coug_fg/fluid_pressure_odom_parameters.hpp"

namespace coug_fg {

class FluidPressureOdomNode : public rclcpp::Node {
 public:
  explicit FluidPressureOdomNode(const rclcpp::NodeOptions& options);

 private:
  // --- Callbacks ---
  void pressureCallback(const sensor_msgs::msg::FluidPressure::ConstSharedPtr& msg);

  void calibrateCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>& request,
                         const std::shared_ptr<std_srvs::srv::Trigger::Response>& response);

  void ahrsCallback(const sensor_msgs::msg::Imu::ConstSharedPtr& msg);

  // --- Helpers ---
  auto resolveOrientation(const std::string& depth_frame) -> geometry_msgs::msg::Quaternion;

  auto convertToOdom(const sensor_msgs::msg::FluidPressure::ConstSharedPtr& msg, double pressure,
                     double reference_pressure, const std::string& depth_frame,
                     const geometry_msgs::msg::Quaternion& map_R_depth) const
      -> nav_msgs::msg::Odometry;

  // --- ROS Interfaces ---
  rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr pressure_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr ahrs_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calibrate_srv_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // --- Parameters ---
  std::shared_ptr<fluid_pressure_odom_node::ParamListener> param_listener_;
  fluid_pressure_odom_node::Params params_;

  // --- State ---
  double last_pressure_{-1.0};
  int rejected_count_{0};
  bool calibrated_{false};
  double calibrated_pressure_{0.0};
  sensor_msgs::msg::Imu::ConstSharedPtr last_ahrs_;
};

}  // namespace coug_fg
