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

#include "coug_fg/dvl_a50_twist_beams.hpp"

#include <rcl/time.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "coug_fg/dvl_a50_twist_beams_parameters.hpp"
#include "coug_interfaces/msg/dvl_beam.hpp"
#include "coug_interfaces/msg/dvl_beam_list.hpp"
#include "dvl_msgs/msg/dvl.hpp"
#include "dvl_msgs/msg/dvl_beam.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "sensor_msgs/msg/range.hpp"

namespace coug_fg {

using coug_interfaces::msg::DvlBeam;
using coug_interfaces::msg::DvlBeamList;

DvlA50TwistBeamsNode::DvlA50TwistBeamsNode(const rclcpp::NodeOptions& options)
    : Node("dvl_a50_twist_beams_node", options) {
  param_listener_ =
      std::make_shared<dvl_a50_twist_beams_node::ParamListener>(get_node_parameters_interface());
  params_ = param_listener_->get_params();

  dvl_sub_ = create_subscription<dvl_msgs::msg::DVL>(
      params_.input_topic, rclcpp::SensorDataQoS(),
      [this](const dvl_msgs::msg::DVL::ConstSharedPtr& msg) { dvlCallback(msg); });

  twist_pub_ = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
      params_.twist_output_topic, rclcpp::SystemDefaultsQoS());

  beams_pub_ =
      create_publisher<DvlBeamList>(params_.beams_output_topic, rclcpp::SystemDefaultsQoS());

  beam_frames_ = {params_.beam0_frame, params_.beam1_frame, params_.beam2_frame,
                  params_.beam3_frame};

  const std::array<std::string, 4> range_topics = {
      params_.beam0_range_topic, params_.beam1_range_topic, params_.beam2_range_topic,
      params_.beam3_range_topic};

  for (size_t i = 0; i < range_pubs_.size(); ++i) {
    range_pubs_[i] =
        create_publisher<sensor_msgs::msg::Range>(range_topics[i], rclcpp::SystemDefaultsQoS());
  }

  RCLCPP_INFO(get_logger(), "Initialization complete.");
}

void DvlA50TwistBeamsNode::dvlCallback(const dvl_msgs::msg::DVL::ConstSharedPtr& msg) {
  const auto now = this->get_clock()->now();
  last_dvl_time_ = now.seconds();

  if (params_.simulate_dropout && params_.dropout_frequency_hz > 0.0) {
    const double cycle_period = 1.0 / params_.dropout_frequency_hz;
    const bool should_drop = std::fmod(last_dvl_time_, cycle_period) < params_.dropout_duration_sec;
    if (should_drop) {
      if (!is_simulating_dropout_) {
        RCLCPP_WARN(get_logger(), "Simulating DVL dropout.");
        is_simulating_dropout_ = true;
      }
      return;
    }
    is_simulating_dropout_ = false;
  }

  if (msg->velocity_valid || msg->fom <= params_.fom_valid_threshold) {
    twist_pub_->publish(convertToTwist(msg));
  } else {
    RCLCPP_WARN(get_logger(), "Received invalid DVL velocity.");
  }

  if (!msg->beams.empty()) {
    beams_pub_->publish(convertToBeams(msg));

    const rclcpp::Time stamp = resolveStamp(msg);
    for (const auto& in : msg->beams) {
      if (in.id < 0 || static_cast<size_t>(in.id) >= range_pubs_.size()) {
        continue;
      }
      range_pubs_[in.id]->publish(convertToRange(in, beam_frames_[in.id], stamp));
    }
  }
}

auto DvlA50TwistBeamsNode::resolveStamp(const dvl_msgs::msg::DVL::ConstSharedPtr& msg) const
    -> rclcpp::Time {
  if (params_.override_timestamp) {
    return {msg->header.stamp};
  }

  static constexpr uint64_t kMicrosecondsPerSecond = 1000000;
  static constexpr uint64_t kNanosecondsPerMicrosecond = 1000;
  const auto sec = static_cast<int32_t>(msg->time_of_validity / kMicrosecondsPerSecond);
  const auto nanosec = static_cast<uint32_t>((msg->time_of_validity % kMicrosecondsPerSecond) *
                                             kNanosecondsPerMicrosecond);
  return {sec, nanosec, RCL_ROS_TIME};
}

auto DvlA50TwistBeamsNode::convertToTwist(const dvl_msgs::msg::DVL::ConstSharedPtr& msg)
    -> geometry_msgs::msg::TwistWithCovarianceStamped {
  geometry_msgs::msg::TwistWithCovarianceStamped twist_msg;
  twist_msg.header.frame_id =
      params_.use_parameter_frame ? params_.parameter_frame : msg->header.frame_id;
  twist_msg.header.stamp = resolveStamp(msg);

  // Convert FRD -> FLU
  static constexpr std::array<double, 3> kFrdToFlu = {1.0, -1.0, -1.0};

  twist_msg.twist.twist.linear.x = kFrdToFlu[0] * msg->velocity.x;
  twist_msg.twist.twist.linear.y = kFrdToFlu[1] * msg->velocity.y;
  twist_msg.twist.twist.linear.z = kFrdToFlu[2] * msg->velocity.z;

  if (params_.use_fom_covariance) {
    const double var_vel = msg->fom * params_.fom_covariance_scale;
    twist_msg.twist.covariance[0] = var_vel;
    twist_msg.twist.covariance[7] = var_vel;
    twist_msg.twist.covariance[14] = var_vel;
  } else {
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        twist_msg.twist.covariance[i * 6 + j] =
            kFrdToFlu[i] * kFrdToFlu[j] * msg->covariance[i * 3 + j];
      }
    }
  }
  return twist_msg;
}

auto DvlA50TwistBeamsNode::convertToBeams(const dvl_msgs::msg::DVL::ConstSharedPtr& msg)
    -> DvlBeamList {
  DvlBeamList beams_msg;
  beams_msg.header.frame_id =
      params_.use_parameter_frame ? params_.parameter_frame : msg->header.frame_id;
  beams_msg.header.stamp = resolveStamp(msg);

  beams_msg.beams.reserve(msg->beams.size());
  for (const auto& in : msg->beams) {
    if (in.id < 0 || static_cast<size_t>(in.id) >= beam_frames_.size()) {
      RCLCPP_WARN(get_logger(), "Received unexpected DVL beam id %ld.", in.id);
      continue;
    }

    DvlBeam beam;
    beam.frame_id = beam_frames_[in.id];
    beam.valid = in.valid;
    beam.velocity = in.velocity;
    beam.distance = in.distance;
    beams_msg.beams.push_back(beam);
  }
  return beams_msg;
}

auto DvlA50TwistBeamsNode::convertToRange(const dvl_msgs::msg::DVLBeam& beam,
                                          const std::string& frame_id,
                                          const rclcpp::Time& stamp) const
    -> sensor_msgs::msg::Range {
  sensor_msgs::msg::Range range_msg;
  range_msg.header.frame_id = frame_id;
  range_msg.header.stamp = stamp;

  range_msg.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
  range_msg.field_of_view = static_cast<float>(params_.beam_field_of_view_radians);
  range_msg.min_range = static_cast<float>(params_.beam_min_range);
  range_msg.max_range = static_cast<float>(params_.beam_max_range);

  range_msg.range =
      beam.valid ? static_cast<float>(beam.distance) : std::numeric_limits<float>::infinity();

  return range_msg;
}

}  // namespace coug_fg

RCLCPP_COMPONENTS_REGISTER_NODE(coug_fg::DvlA50TwistBeamsNode)
