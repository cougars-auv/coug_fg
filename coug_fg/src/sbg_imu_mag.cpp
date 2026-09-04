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

#include "coug_fg/sbg_imu_mag.hpp"

#include <functional>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "coug_fg/sbg_imu_mag_parameters.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"

namespace coug_fg {

SbgImuMagNode::SbgImuMagNode(const rclcpp::NodeOptions& options)
    : Node("sbg_imu_mag_node", options) {
  param_listener_ =
      std::make_shared<sbg_imu_mag_node::ParamListener>(get_node_parameters_interface());
  params_ = param_listener_->get_params();

  mag_sub_ = create_subscription<sensor_msgs::msg::MagneticField>(
      params_.input_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::MagneticField::SharedPtr msg) { magCallback(msg); });

  mag_pub_ = create_publisher<sensor_msgs::msg::MagneticField>(params_.output_topic,
                                                               rclcpp::SystemDefaultsQoS());

  RCLCPP_INFO(get_logger(), "Initialization complete.");
}

void SbgImuMagNode::magCallback(const sensor_msgs::msg::MagneticField::SharedPtr& msg) {
  mag_pub_->publish(convertToTesla(msg));
}

auto SbgImuMagNode::convertToTesla(const sensor_msgs::msg::MagneticField::SharedPtr& msg) const
    -> sensor_msgs::msg::MagneticField {
  sensor_msgs::msg::MagneticField mag_msg = *msg;

  const double scale = params_.au_to_tesla;

  mag_msg.magnetic_field.x = msg->magnetic_field.x * scale;
  mag_msg.magnetic_field.y = msg->magnetic_field.y * scale;
  mag_msg.magnetic_field.z = msg->magnetic_field.z * scale;

  if (mag_msg.magnetic_field_covariance[0] >= 0.0) {
    for (auto& element : mag_msg.magnetic_field_covariance) {
      element *= scale * scale;
    }
  }

  return mag_msg;
}

}  // namespace coug_fg

RCLCPP_COMPONENTS_REGISTER_NODE(coug_fg::SbgImuMagNode)
