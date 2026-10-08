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

#include "coug_fg/seatrac_x150_imu_depth.hpp"

#include <Eigen/Core>
#include <cmath>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "coug_fg/seatrac_x150_imu_depth_parameters.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "seatrac_interfaces/msg/modem_status.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace coug_fg {

namespace {

constexpr double kUnknownCovariance = -1.0;

}  // namespace

SeatracX150ImuDepthNode::SeatracX150ImuDepthNode(const rclcpp::NodeOptions& options)
    : Node("seatrac_x150_imu_depth_node", options) {
  param_listener_ =
      std::make_shared<seatrac_x150_imu_depth_node::ParamListener>(get_node_parameters_interface());
  params_ = param_listener_->get_params();

  modem_sub_ = create_subscription<seatrac_interfaces::msg::ModemStatus>(
      params_.input_topic, rclcpp::SensorDataQoS(),
      [this](const seatrac_interfaces::msg::ModemStatus::ConstSharedPtr& msg) {
        modemStatusCallback(msg);
      });

  ahrs_pub_ = create_publisher<sensor_msgs::msg::Imu>(params_.ahrs_output_topic,
                                                      rclcpp::SystemDefaultsQoS());

  depth_pub_ = create_publisher<nav_msgs::msg::Odometry>(params_.depth_output_topic,
                                                         rclcpp::SystemDefaultsQoS());

  RCLCPP_INFO(get_logger(), "Initialization complete.");
}

void SeatracX150ImuDepthNode::modemStatusCallback(
    const seatrac_interfaces::msg::ModemStatus::ConstSharedPtr& msg) {
  if (msg->includes_local_attitude) {
    ahrs_pub_->publish(convertToAhrs(msg));
  }

  if (msg->includes_env_fields) {
    depth_pub_->publish(convertToOdom(msg));
  }
}

auto SeatracX150ImuDepthNode::convertToAhrs(
    const seatrac_interfaces::msg::ModemStatus::ConstSharedPtr& msg) const
    -> sensor_msgs::msg::Imu {
  sensor_msgs::msg::Imu ahrs_msg;
  ahrs_msg.header = msg->header;
  if (params_.use_parameter_frame) {
    ahrs_msg.header.frame_id = params_.parameter_frame;
  }

  static constexpr double kSeatracToRad = M_PI / 1800.0;
  const double roll_rad = msg->attitude_roll * kSeatracToRad;
  const double pitch_rad = msg->attitude_pitch * kSeatracToRad;
  const double yaw_rad = msg->attitude_yaw * kSeatracToRad + params_.mag_declination_radians;

  tf2::Quaternion q;
  q.setRPY(roll_rad, pitch_rad, yaw_rad);

  // Convert FRD -> FLU
  static const tf2::Quaternion kFrdToFlu(1.0, 0.0, 0.0, 0.0);
  q *= kFrdToFlu;

  // Convert NED -> ENU
  static const tf2::Quaternion kNedToEnu(M_SQRT1_2, M_SQRT1_2, 0.0, 0.0);
  ahrs_msg.orientation = tf2::toMsg(kNedToEnu * q);

  const auto& sigmas = params_.orientation_noise_sigmas;
  ahrs_msg.orientation_covariance[0] = sigmas[0] * sigmas[0];
  ahrs_msg.orientation_covariance[4] = sigmas[1] * sigmas[1];
  ahrs_msg.orientation_covariance[8] = sigmas[2] * sigmas[2];

  // IMU orientation covariance is expressed about the world-frame axes
  static const Eigen::Matrix3d kNedToEnu3D =
      (Eigen::Matrix3d() << 0, 1, 0, 1, 0, 0, 0, 0, -1).finished();
  Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> cov(
      ahrs_msg.orientation_covariance.data());
  cov = (kNedToEnu3D * cov * kNedToEnu3D.transpose()).eval();

  ahrs_msg.linear_acceleration_covariance[0] = kUnknownCovariance;
  ahrs_msg.angular_velocity_covariance[0] = kUnknownCovariance;

  return ahrs_msg;
}

auto SeatracX150ImuDepthNode::convertToOdom(
    const seatrac_interfaces::msg::ModemStatus::ConstSharedPtr& msg) const
    -> nav_msgs::msg::Odometry {
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = msg->header.stamp;
  odom_msg.header.frame_id = params_.map_frame;

  odom_msg.child_frame_id =
      params_.use_parameter_frame ? params_.parameter_frame : msg->header.frame_id;

  static constexpr double kSeatracToMeters = 0.1;
  odom_msg.pose.pose.position.z = msg->depth_local * kSeatracToMeters;

  const double var_depth = params_.position_z_noise_sigma * params_.position_z_noise_sigma;
  odom_msg.pose.covariance[14] = var_depth;

  static constexpr double kUnmeasuredVariance = 1e9;
  odom_msg.pose.covariance[0] = kUnmeasuredVariance;
  odom_msg.pose.covariance[7] = kUnmeasuredVariance;
  odom_msg.pose.covariance[21] = kUnmeasuredVariance;
  odom_msg.pose.covariance[28] = kUnmeasuredVariance;
  odom_msg.pose.covariance[35] = kUnmeasuredVariance;

  odom_msg.twist.covariance[0] = kUnknownCovariance;

  // Convert NED -> ENU
  static const geometry_msgs::msg::TransformStamped kNedToEnu = []() {
    geometry_msgs::msg::TransformStamped transform;
    transform.transform.rotation = tf2::toMsg(tf2::Quaternion(M_SQRT1_2, M_SQRT1_2, 0.0, 0.0));
    return transform;
  }();

  // Pose orientation covariance is expressed about the world-frame axes
  const geometry_msgs::msg::PoseWithCovariance ned_pose = odom_msg.pose;
  tf2::doTransform(ned_pose, odom_msg.pose, kNedToEnu);

  return odom_msg;
}

}  // namespace coug_fg

RCLCPP_COMPONENTS_REGISTER_NODE(coug_fg::SeatracX150ImuDepthNode)
