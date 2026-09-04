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

#include "coug_fg/dvl_a50_twist_beams_parameters.hpp"

namespace coug_fg {

class DvlA50TwistBeamsNode : public rclcpp::Node {
 public:
  explicit DvlA50TwistBeamsNode(const rclcpp::NodeOptions& options);

 private:
  // --- Callbacks ---
  void dvlCallback(const dvl_msgs::msg::DVL::ConstSharedPtr& msg);

  // --- Helpers ---
  rclcpp::Time resolveStamp(const dvl_msgs::msg::DVL::ConstSharedPtr& msg) const;

  geometry_msgs::msg::TwistWithCovarianceStamped convertToTwist(
      const dvl_msgs::msg::DVL::ConstSharedPtr& msg);

  coug_interfaces::msg::DvlBeamList convertToBeams(const dvl_msgs::msg::DVL::ConstSharedPtr& msg);

  sensor_msgs::msg::Range convertToRange(const dvl_msgs::msg::DVLBeam& beam,
                                         const std::string& frame_id,
                                         const rclcpp::Time& stamp) const;

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

}  // namespace coug_fg
