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

#include "coug_fg/navsat_odom.hpp"

#include <Eigen/Core>
#include <GeographicLib/Geocentric.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <diagnostic_updater/diagnostic_status_wrapper.hpp>
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

#include "coug_fg/navsat_odom_parameters.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"

namespace coug_fg {

NavsatOdomNode::NavsatOdomNode(const rclcpp::NodeOptions& options)
    : Node("navsat_odom_node", options),
      diagnostic_updater_(this),
      local_cartesian_(0.0, 0.0, 0.0, GeographicLib::Geocentric::WGS84()) {
  param_listener_ =
      std::make_shared<navsat_odom_node::ParamListener>(get_node_parameters_interface());
  params_ = param_listener_->get_params();

  navsat_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      params_.input_topic, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::NavSatFix::ConstSharedPtr& msg) { navsatCallback(msg); });

  odom_pub_ =
      create_publisher<nav_msgs::msg::Odometry>(params_.output_topic, rclcpp::SystemDefaultsQoS());

  if (params_.use_ahrs) {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    ahrs_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        params_.ahrs_topic, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Imu::ConstSharedPtr& msg) { ahrsCallback(msg); });
  }

  if (params_.set_origin) {
    origin_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>(params_.origin_topic,
                                                                rclcpp::SystemDefaultsQoS());
    origin_timer_ = create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / params_.origin_pub_rate_hz)), [this]() {
          if (origin_set_) {
            origin_pub_->publish(origin_navsat_);
          }
        });
  } else {
    origin_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
        params_.origin_topic, rclcpp::SystemDefaultsQoS(),
        [this](const sensor_msgs::msg::NavSatFix::ConstSharedPtr& msg) { originCallback(msg); });
  }

  if (params_.set_origin && params_.use_parameter_origin) {
    sensor_msgs::msg::NavSatFix origin;
    origin.header.frame_id = params_.map_frame;
    origin.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    origin.latitude = params_.origin_latitude;
    origin.longitude = params_.origin_longitude;
    origin.altitude = params_.origin_altitude;
    setOrigin(origin);
    RCLCPP_INFO(get_logger(), "Origin set from parameters: lat %.6f, lon %.6f, alt %.2f m.",
                origin_navsat_.latitude, origin_navsat_.longitude, origin_navsat_.altitude);
  }

  if (params_.publish_diagnostics) {
    const std::string ns = this->get_namespace();
    const std::string clean_ns = (ns == "/") ? "" : ns;
    diagnostic_updater_.setHardwareID(clean_ns + "/navsat_odom_node");

    const std::string prefix = clean_ns.empty() ? "" : "[" + clean_ns + "] ";

    const std::string origin_task = prefix + "Origin Status";
    diagnostic_updater_.add(origin_task, [this](diagnostic_updater::DiagnosticStatusWrapper& stat) {
      checkOriginStatus(stat);
    });
  }

  RCLCPP_INFO(get_logger(), "Initialization complete.");
}

void NavsatOdomNode::originCallback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr& msg) {
  if (!origin_set_ && msg->status.status >= sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
    setOrigin(*msg);
    RCLCPP_INFO(get_logger(), "Origin set from origin topic: lat %.6f, lon %.6f, alt %.2f m.",
                origin_navsat_.latitude, origin_navsat_.longitude, origin_navsat_.altitude);
  }
}

void NavsatOdomNode::navsatCallback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr& msg) {
  if (msg->status.status == sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX) {
    RCLCPP_WARN(get_logger(), "Rejected GPS fix: no fix.");
    return;
  }

  if (!origin_set_) {
    if (!params_.set_origin) {
      return;
    }
    setOrigin(*msg);
    RCLCPP_INFO(get_logger(), "Origin set from first GPS fix: lat %.6f, lon %.6f, alt %.2f m.",
                origin_navsat_.latitude, origin_navsat_.longitude, origin_navsat_.altitude);
    return;
  }

  if (msg->position_covariance_type == sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN) {
    RCLCPP_ERROR(get_logger(), "Rejected GPS fix: position covariance type is unknown.");
    return;
  }

  // Reject degraded fixes (e.g. when using GPS as ground truth)
  if (params_.position_covariance_threshold > 0.0 &&
      std::max(msg->position_covariance[0], msg->position_covariance[4]) >
          params_.position_covariance_threshold) {
    RCLCPP_WARN(get_logger(), "Rejected GPS fix: horizontal variance %.2f m^2 exceeds %.2f m^2.",
                std::max(msg->position_covariance[0], msg->position_covariance[4]),
                params_.position_covariance_threshold);
    return;
  }

  const std::string gps_frame =
      params_.use_parameter_child_frame ? params_.parameter_child_frame : msg->header.frame_id;

  geometry_msgs::msg::Quaternion map_R_gps;
  if (params_.use_ahrs) {
    map_R_gps = resolveOrientation(gps_frame);
  }

  odom_pub_->publish(convertToOdom(msg, gps_frame, map_R_gps));
}

void NavsatOdomNode::ahrsCallback(const sensor_msgs::msg::Imu::ConstSharedPtr& msg) {
  const double var = msg->orientation_covariance[0];
  if (std::isfinite(var) && var > 0.0) {
    last_ahrs_ = msg;
  }
}

void NavsatOdomNode::setOrigin(const sensor_msgs::msg::NavSatFix& msg) {
  local_cartesian_.Reset(msg.latitude, msg.longitude, msg.altitude);
  origin_navsat_ = msg;
  if (origin_navsat_.header.frame_id.empty()) {
    origin_navsat_.header.frame_id = params_.map_frame;
  }
  origin_set_ = true;
}

auto NavsatOdomNode::resolveOrientation(const std::string& gps_frame)
    -> geometry_msgs::msg::Quaternion {
  geometry_msgs::msg::Quaternion map_R_gps;

  if (!last_ahrs_) {
    return map_R_gps;
  }

  geometry_msgs::msg::TransformStamped ahrs_T_gps_tf;
  try {
    ahrs_T_gps_tf =
        tf_buffer_->lookupTransform(last_ahrs_->header.frame_id, gps_frame, tf2::TimePointZero);
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "Failed to look up transform from '%s' to '%s': %s", gps_frame.c_str(),
                         last_ahrs_->header.frame_id.c_str(), ex.what());
    return map_R_gps;
  }

  tf2::Quaternion map_R_ahrs;
  tf2::Quaternion ahrs_R_gps;
  tf2::fromMsg(last_ahrs_->orientation, map_R_ahrs);
  tf2::fromMsg(ahrs_T_gps_tf.transform.rotation, ahrs_R_gps);

  return tf2::toMsg((map_R_ahrs * ahrs_R_gps).normalized());
}

auto NavsatOdomNode::convertToOdom(const sensor_msgs::msg::NavSatFix::ConstSharedPtr& msg,
                                   const std::string& gps_frame,
                                   const geometry_msgs::msg::Quaternion& map_R_gps)
    -> nav_msgs::msg::Odometry {
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = msg->header.stamp;
  odom_msg.header.frame_id = params_.map_frame;
  odom_msg.child_frame_id = gps_frame;

  double east = 0.0;
  double north = 0.0;
  double up = 0.0;
  local_cartesian_.Forward(msg->latitude, msg->longitude, msg->altitude, east, north, up);

  odom_msg.pose.pose.position.x = east;
  odom_msg.pose.pose.position.y = north;
  odom_msg.pose.pose.position.z = up;

  odom_msg.pose.pose.orientation = map_R_gps;

  const Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> cov(
      msg->position_covariance.data());
  Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(odom_msg.pose.covariance.data())
      .topLeftCorner<3, 3>() = cov;

  static constexpr double kUnmeasuredVariance = 1e9;
  odom_msg.pose.covariance[21] = kUnmeasuredVariance;
  odom_msg.pose.covariance[28] = kUnmeasuredVariance;
  odom_msg.pose.covariance[35] = kUnmeasuredVariance;

  static constexpr double kUnknownCovariance = -1.0;
  odom_msg.twist.covariance[0] = kUnknownCovariance;

  return odom_msg;
}

void NavsatOdomNode::checkOriginStatus(diagnostic_updater::DiagnosticStatusWrapper& stat) const {
  if (origin_set_) {
    stat.add("Origin Latitude", origin_navsat_.latitude);
    stat.add("Origin Longitude", origin_navsat_.longitude);
    stat.add("Origin Altitude", origin_navsat_.altitude);
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Origin successfully set.");
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Waiting for origin.");
  }
}

}  // namespace coug_fg

RCLCPP_COMPONENTS_REGISTER_NODE(coug_fg::NavsatOdomNode)
