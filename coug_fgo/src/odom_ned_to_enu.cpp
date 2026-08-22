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

#include "coug_fgo/odom_ned_to_enu.hpp"

#include <tf2/LinearMath/Quaternion.h>

#include <Eigen/Core>
#include <cmath>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace coug_fgo {

OdomNedToEnuNode::OdomNedToEnuNode(const rclcpp::NodeOptions& options)
    : Node("odom_ned_to_enu_node", options) {
  param_listener_ =
      std::make_shared<odom_ned_to_enu_node::ParamListener>(get_node_parameters_interface());
  params_ = param_listener_->get_params();

  // --- ROS Interfaces ---
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      params_.input_topic, rclcpp::SensorDataQoS(),
      std::bind(&OdomNedToEnuNode::odomCallback, this, std::placeholders::_1));

  odom_pub_ =
      create_publisher<nav_msgs::msg::Odometry>(params_.output_topic, rclcpp::SystemDefaultsQoS());

  RCLCPP_INFO(get_logger(), "Initialization complete.");
}

void OdomNedToEnuNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  odom_pub_->publish(convertToEnu(msg));
}

nav_msgs::msg::Odometry OdomNedToEnuNode::convertToEnu(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  nav_msgs::msg::Odometry out = *msg;

  // Convert NED -> ENU
  static const tf2::Quaternion kNedToEnu(M_SQRT1_2, M_SQRT1_2, 0.0, 0.0);

  const auto& p = msg->pose.pose.position;
  tf2::Vector3 position = tf2::quatRotate(kNedToEnu, tf2::Vector3(p.x, p.y, p.z));
  out.pose.pose.position.x = position.x();
  out.pose.pose.position.y = position.y();
  out.pose.pose.position.z = position.z();

  tf2::Quaternion q;
  tf2::fromMsg(msg->pose.pose.orientation, q);
  out.pose.pose.orientation = tf2::toMsg(kNedToEnu * q);

  if (out.pose.covariance[0] >= 0.0) {
    // Pose orientation covariance is expressed about the world-frame axes
    static const Eigen::Matrix<double, 6, 6> kNedToEnu6D = []() {
      static const Eigen::Matrix3d kNedToEnu3D =
          (Eigen::Matrix3d() << 0, 1, 0, 1, 0, 0, 0, 0, -1).finished();
      Eigen::Matrix<double, 6, 6> t = Eigen::Matrix<double, 6, 6>::Zero();
      t.block<3, 3>(0, 0) = kNedToEnu3D;
      t.block<3, 3>(3, 3) = kNedToEnu3D;
      return t;
    }();
    Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> cov(out.pose.covariance.data());
    cov = (kNedToEnu6D * cov * kNedToEnu6D.transpose()).eval();
  }

  return out;
}

}  // namespace coug_fgo

RCLCPP_COMPONENTS_REGISTER_NODE(coug_fgo::OdomNedToEnuNode)
