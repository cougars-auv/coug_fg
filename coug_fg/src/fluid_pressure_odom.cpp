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

#include "coug_fg/fluid_pressure_odom.hpp"

#include <cmath>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <string>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/convert.hpp>
#include <tf2/exceptions.hpp>
#include <tf2/time.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include "coug_fg/fluid_pressure_odom_parameters.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace coug_fg {

FluidPressureOdomNode::FluidPressureOdomNode(const rclcpp::NodeOptions& options)
    : Node("fluid_pressure_odom_node", options) {
  param_listener_ =
      std::make_shared<fluid_pressure_odom_node::ParamListener>(get_node_parameters_interface());
  params_ = param_listener_->get_params();

  pressure_sub_ = create_subscription<sensor_msgs::msg::FluidPressure>(
      params_.input_topic, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::FluidPressure::ConstSharedPtr& msg) {
        pressureCallback(msg);
      });

  odom_pub_ =
      create_publisher<nav_msgs::msg::Odometry>(params_.output_topic, rclcpp::SystemDefaultsQoS());

  if (params_.use_ahrs) {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    ahrs_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        params_.ahrs_topic, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Imu::ConstSharedPtr& msg) { ahrsCallback(msg); });
  }

  calibrate_srv_ = create_service<std_srvs::srv::Trigger>(
      params_.calibrate_service,
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>& request,
             const std::shared_ptr<std_srvs::srv::Trigger::Response>& response) {
        calibrateCallback(request, response);
      });

  RCLCPP_INFO(get_logger(), "Initialization complete.");
}

void FluidPressureOdomNode::pressureCallback(
    const sensor_msgs::msg::FluidPressure::ConstSharedPtr& msg) {
  const double pressure = msg->fluid_pressure * params_.pressure_scale;

  if (params_.max_pressure_delta > 0.0 && last_pressure_ >= 0.0 &&
      std::abs(pressure - last_pressure_) > params_.max_pressure_delta) {
    rejected_count_++;
    if (rejected_count_ <= params_.max_consecutive_rejections) {
      RCLCPP_WARN(get_logger(), "Rejected pressure spike.");
      return;
    }
    RCLCPP_WARN(get_logger(), "Accepting pressure step after %d consecutive rejections.",
                rejected_count_);
  }
  rejected_count_ = 0;
  last_pressure_ = pressure;

  const double reference_pressure =
      calibrated_ ? calibrated_pressure_ : params_.atmospheric_pressure;

  const std::string depth_frame =
      params_.use_parameter_child_frame ? params_.parameter_child_frame : msg->header.frame_id;

  geometry_msgs::msg::Quaternion map_R_depth;
  if (params_.use_ahrs) {
    map_R_depth = resolveOrientation(depth_frame);
  }

  odom_pub_->publish(convertToOdom(msg, pressure, reference_pressure, depth_frame, map_R_depth));
}

void FluidPressureOdomNode::calibrateCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>& request,
    const std::shared_ptr<std_srvs::srv::Trigger::Response>& response) {
  (void)request;

  if (last_pressure_ < 0.0) {
    response->success = false;
    response->message = "No pressure data.";
    return;
  }

  calibrated_pressure_ = last_pressure_;
  calibrated_ = true;
  rejected_count_ = 0;

  response->success = true;
  response->message = "Depth calibrated.";
  RCLCPP_INFO(get_logger(), "Depth calibrated: zero reference set to %.1f Pa.",
              calibrated_pressure_);
}

void FluidPressureOdomNode::ahrsCallback(const sensor_msgs::msg::Imu::ConstSharedPtr& msg) {
  const double var = msg->orientation_covariance[0];
  if (std::isfinite(var) && var > 0.0) {
    last_ahrs_ = msg;
  }
}

auto FluidPressureOdomNode::resolveOrientation(const std::string& depth_frame)
    -> geometry_msgs::msg::Quaternion {
  geometry_msgs::msg::Quaternion map_R_depth;

  if (!last_ahrs_) {
    return map_R_depth;
  }

  geometry_msgs::msg::TransformStamped ahrs_T_depth_tf;
  try {
    ahrs_T_depth_tf =
        tf_buffer_->lookupTransform(last_ahrs_->header.frame_id, depth_frame, tf2::TimePointZero);
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Could not transform %s to %s: %s",
                         last_ahrs_->header.frame_id.c_str(), depth_frame.c_str(), ex.what());
    return map_R_depth;
  }

  tf2::Quaternion map_R_ahrs;
  tf2::Quaternion ahrs_R_depth;
  tf2::fromMsg(last_ahrs_->orientation, map_R_ahrs);
  tf2::fromMsg(ahrs_T_depth_tf.transform.rotation, ahrs_R_depth);

  return tf2::toMsg((map_R_ahrs * ahrs_R_depth).normalized());
}

auto FluidPressureOdomNode::convertToOdom(
    const sensor_msgs::msg::FluidPressure::ConstSharedPtr& msg, double pressure,
    double reference_pressure, const std::string& depth_frame,
    const geometry_msgs::msg::Quaternion& map_R_depth) const -> nav_msgs::msg::Odometry {
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = msg->header.stamp;
  odom_msg.header.frame_id = params_.map_frame;

  odom_msg.child_frame_id = depth_frame;

  // depth [m] = (pressure [Pa] - reference_pressure [Pa]) / (water_density [kg/m^3] * g [m/s^2])
  const double pressure_to_depth = 1.0 / (params_.water_density * params_.gravity);
  const double gauge_pressure = pressure - reference_pressure;
  odom_msg.pose.pose.position.z = -gauge_pressure * pressure_to_depth;
  odom_msg.pose.pose.orientation = map_R_depth;

  // var_depth = var_pressure / (rho*g)^2
  const double var_pressure = msg->variance * params_.pressure_scale * params_.pressure_scale;
  const double var_depth = var_pressure * pressure_to_depth * pressure_to_depth;
  odom_msg.pose.covariance[14] = var_depth;

  static constexpr double kUnmeasuredVariance = 1e9;
  odom_msg.pose.covariance[0] = kUnmeasuredVariance;
  odom_msg.pose.covariance[7] = kUnmeasuredVariance;
  odom_msg.pose.covariance[21] = kUnmeasuredVariance;
  odom_msg.pose.covariance[28] = kUnmeasuredVariance;
  odom_msg.pose.covariance[35] = kUnmeasuredVariance;

  static constexpr double kUnknownCovariance = -1.0;
  odom_msg.twist.covariance[0] = kUnknownCovariance;

  return odom_msg;
}

}  // namespace coug_fg

RCLCPP_COMPONENTS_REGISTER_NODE(coug_fg::FluidPressureOdomNode)
