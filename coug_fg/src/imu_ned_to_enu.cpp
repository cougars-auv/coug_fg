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

#include "coug_fg/imu_ned_to_enu.hpp"

#include <Eigen/Dense>
#include <cmath>
#include <functional>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "coug_fg/imu_ned_to_enu_parameters.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace coug_fg {

ImuNedToEnuNode::ImuNedToEnuNode(const rclcpp::NodeOptions& options)
    : Node("imu_ned_to_enu_node", options) {
  param_listener_ =
      std::make_shared<imu_ned_to_enu_node::ParamListener>(get_node_parameters_interface());
  params_ = param_listener_->get_params();

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      params_.input_topic, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Imu::ConstSharedPtr& msg) { imuCallback(msg); });

  imu_pub_ =
      create_publisher<sensor_msgs::msg::Imu>(params_.output_topic, rclcpp::SystemDefaultsQoS());

  RCLCPP_INFO(get_logger(), "Initialization complete.");
}

void ImuNedToEnuNode::imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr& msg) {
  imu_pub_->publish(convertToEnu(msg));
}

auto ImuNedToEnuNode::convertToEnu(const sensor_msgs::msg::Imu::ConstSharedPtr& msg)
    -> sensor_msgs::msg::Imu {
  sensor_msgs::msg::Imu imu_msg = *msg;

  // Convert NED -> ENU
  static const tf2::Quaternion kNedToEnu(M_SQRT1_2, M_SQRT1_2, 0.0, 0.0);

  tf2::Quaternion q;
  tf2::fromMsg(msg->orientation, q);
  imu_msg.orientation = tf2::toMsg(kNedToEnu * q);

  if (imu_msg.orientation_covariance[0] >= 0.0) {
    // IMU orientation covariance is expressed about the world-frame axes
    static const Eigen::Matrix3d kNedToEnu3D =
        (Eigen::Matrix3d() << 0, 1, 0, 1, 0, 0, 0, 0, -1).finished();
    Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> cov(
        imu_msg.orientation_covariance.data());
    cov = (kNedToEnu3D * cov * kNedToEnu3D.transpose()).eval();
  }

  return imu_msg;
}

}  // namespace coug_fg

RCLCPP_COMPONENTS_REGISTER_NODE(coug_fg::ImuNedToEnuNode)
