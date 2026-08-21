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

/**
 * @file dvl_a50_twist_beams.hpp
 * @brief ROS 2 node that converts DVL A50 velocity data to twist and beam messages.
 * @author Nelson Durrant
 * @date May 2026
 */

#pragma once

#include <array>
#include <coug_interfaces/msg/dvl_beam_list.hpp>
#include <dvl_msgs/msg/dvl.hpp>
#include <dvl_msgs/msg/dvl_beam.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <string>

#include "coug_fgo/dvl_a50_twist_beams_parameters.hpp"

namespace coug_fgo {

/**
 * @class DvlA50TwistBeamsNode
 * @brief ROS 2 node that converts DVL A50 velocity data to twist and beam messages.
 */
class DvlA50TwistBeamsNode : public rclcpp::Node {
 public:
  /**
   * @brief Constructs the node and sets up the DVL velocity and beam conversions.
   * @param options The node options.
   */
  explicit DvlA50TwistBeamsNode(const rclcpp::NodeOptions& options);

 private:
  /**
   * @brief Gates DVL samples on simulated dropout, then publishes the twist and/or beam outputs.
   * @param msg The incoming DVL message.
   */
  void dvlCallback(const dvl_msgs::msg::DVL::SharedPtr msg);

  /**
   * @brief Resolves the output stamp from the A50 time-of-validity or the message header.
   * @param msg The incoming DVL message.
   * @return The time-of-validity stamp, or the header stamp if override_timestamp is set.
   */
  rclcpp::Time resolveStamp(const dvl_msgs::msg::DVL::SharedPtr msg);

  /**
   * @brief Converts a DVL report to a stamped twist with FOM- or message-derived covariance.
   * @param msg The incoming DVL message.
   * @return The converted TwistWithCovarianceStamped message (time-of-validity or header stamp).
   */
  geometry_msgs::msg::TwistWithCovarianceStamped convertToTwist(
      const dvl_msgs::msg::DVL::SharedPtr msg);

  /**
   * @brief Converts the per-transducer returns of a DVL report to a beam list.
   * @param msg The incoming DVL message.
   * @return The converted DvlBeamList message; each beam is stamped with its parameter frame.
   */
  coug_interfaces::msg::DvlBeamList convertToBeams(const dvl_msgs::msg::DVL::SharedPtr msg);

  /**
   * @brief Converts a single transducer return of a DVL report to a beam-frame range.
   * @param beam The incoming DVL beam (distance in meters).
   * @param frame_id The beam frame, oriented +x down the beam.
   * @param stamp The resolved output stamp, shared by every beam of the report.
   * @return The converted Range message; a lost beam reports an infinite range.
   */
  sensor_msgs::msg::Range convertToRange(const dvl_msgs::msg::DVLBeam& beam,
                                         const std::string& frame_id, const rclcpp::Time& stamp);

  // --- ROS Interfaces ---
  rclcpp::Subscription<dvl_msgs::msg::DVL>::SharedPtr dvl_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<coug_interfaces::msg::DvlBeamList>::SharedPtr beams_pub_;
  std::array<rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr, 4> range_pubs_;

  // --- Parameters ---
  std::shared_ptr<dvl_a50_twist_beams_node::ParamListener> param_listener_;
  dvl_a50_twist_beams_node::Params params_;
  std::array<std::string, 4> beam_frames_;

  // --- State ---
  double last_dvl_time_{0.0};
  bool is_simulating_dropout_{false};
};

}  // namespace coug_fgo
