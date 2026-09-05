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

#include "coug_fg/odom_ned_to_enu.hpp"

#include <Eigen/Core>
#include <cmath>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Vector3.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "coug_fg/odom_ned_to_enu_parameters.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace coug_fg {

OdomNedToEnuNode::OdomNedToEnuNode(rclcpp::NodeOptions const& options)
    : Node("odom_ned_to_enu_node", options) {
  param_listener_ =
      std::make_shared<odom_ned_to_enu_node::ParamListener>(get_node_parameters_interface());
  params_ = param_listener_->get_params();

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      params_.input_topic, rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr const& msg) { odomCallback(msg); });

  odom_pub_ =
      create_publisher<nav_msgs::msg::Odometry>(params_.output_topic, rclcpp::SystemDefaultsQoS());

  RCLCPP_INFO(get_logger(), "Initialization complete.");
}

void OdomNedToEnuNode::odomCallback(nav_msgs::msg::Odometry::ConstSharedPtr const& msg) {
  odom_pub_->publish(convertToEnu(msg));
}

auto OdomNedToEnuNode::convertToEnu(nav_msgs::msg::Odometry::ConstSharedPtr const& msg)
    -> nav_msgs::msg::Odometry {
  nav_msgs::msg::Odometry odom_msg = *msg;

  // Convert NED -> ENU
  static tf2::Quaternion const kNedToEnu(M_SQRT1_2, M_SQRT1_2, 0.0, 0.0);

  auto const& ned_position = msg->pose.pose.position;
  tf2::Vector3 const enu_position =
      tf2::quatRotate(kNedToEnu, tf2::Vector3(ned_position.x, ned_position.y, ned_position.z));
  odom_msg.pose.pose.position.x = enu_position.x();
  odom_msg.pose.pose.position.y = enu_position.y();
  odom_msg.pose.pose.position.z = enu_position.z();

  tf2::Quaternion q;
  tf2::fromMsg(msg->pose.pose.orientation, q);
  odom_msg.pose.pose.orientation = tf2::toMsg(kNedToEnu * q);

  if (odom_msg.pose.covariance[0] >= 0.0) {
    // Pose orientation covariance is expressed about the world-frame axes
    static Eigen::Matrix<double, 6, 6> const kNedToEnu6D = []() {
      static Eigen::Matrix3d const kNedToEnu3D =
          (Eigen::Matrix3d() << 0, 1, 0, 1, 0, 0, 0, 0, -1).finished();
      Eigen::Matrix<double, 6, 6> transform = Eigen::Matrix<double, 6, 6>::Zero();
      transform.block<3, 3>(0, 0) = kNedToEnu3D;
      transform.block<3, 3>(3, 3) = kNedToEnu3D;
      return transform;
    }();
    Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> covariance(
        odom_msg.pose.covariance.data());
    covariance = (kNedToEnu6D * covariance * kNedToEnu6D.transpose()).eval();
  }

  return odom_msg;
}

}  // namespace coug_fg

RCLCPP_COMPONENTS_REGISTER_NODE(coug_fg::OdomNedToEnuNode)
