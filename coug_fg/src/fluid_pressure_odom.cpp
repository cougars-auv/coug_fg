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

#include "coug_fg/fluid_pressure_odom.hpp"

#include <cmath>
#include <functional>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "coug_fg/fluid_pressure_odom_parameters.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
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
  double const pressure = msg->fluid_pressure * params_.pressure_scale;

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

  double const reference_pressure =
      calibrated_ ? calibrated_pressure_ : params_.atmospheric_pressure;
  odom_pub_->publish(convertToOdom(msg, pressure, reference_pressure));
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

auto FluidPressureOdomNode::convertToOdom(
    const sensor_msgs::msg::FluidPressure::ConstSharedPtr& msg, double pressure,
    double reference_pressure) const -> nav_msgs::msg::Odometry {
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = msg->header.stamp;
  odom_msg.header.frame_id = params_.map_frame;

  odom_msg.child_frame_id =
      params_.use_parameter_child_frame ? params_.parameter_child_frame : msg->header.frame_id;

  // depth [m] = (pressure [Pa] - reference_pressure [Pa]) / (water_density [kg/m^3] * g [m/s^2])
  double const pressure_to_depth = 1.0 / (params_.water_density * params_.gravity);
  double const gauge_pressure = pressure - reference_pressure;
  odom_msg.pose.pose.position.z = -gauge_pressure * pressure_to_depth;
  odom_msg.pose.pose.orientation.w = 1.0;

  // var_depth = var_pressure / (rho*g)^2
  double const var_pressure = msg->variance * params_.pressure_scale * params_.pressure_scale;
  double const var_depth = var_pressure * pressure_to_depth * pressure_to_depth;
  odom_msg.pose.covariance[14] = var_depth;

  return odom_msg;
}

}  // namespace coug_fg

RCLCPP_COMPONENTS_REGISTER_NODE(coug_fg::FluidPressureOdomNode)
