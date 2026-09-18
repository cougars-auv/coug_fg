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

#include "coug_fg/odom_to_tf.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <string>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

#include "coug_fg/odom_to_tf_parameters.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace coug_fg {

OdomToTfNode::OdomToTfNode(const rclcpp::NodeOptions& options) : Node("odom_to_tf_node", options) {
  param_listener_ =
      std::make_shared<odom_to_tf_node::ParamListener>(get_node_parameters_interface());
  params_ = param_listener_->get_params();

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      params_.input_topic, rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::ConstSharedPtr& msg) { odomCallback(msg); });

  RCLCPP_INFO(get_logger(), "Initialization complete.");
}

void OdomToTfNode::odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& msg) {
  tf_broadcaster_->sendTransform(convertToTf(msg));
}

auto OdomToTfNode::convertToTf(const nav_msgs::msg::Odometry::ConstSharedPtr& msg) const
    -> geometry_msgs::msg::TransformStamped {
  const std::string& parent_frame =
      params_.use_parameter_frame ? params_.parameter_frame : msg->header.frame_id;

  tf2::Transform parent_T_child;
  tf2::fromMsg(msg->pose.pose, parent_T_child);

  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = msg->header.stamp;

  if (params_.invert) {
    tf_msg.header.frame_id = msg->child_frame_id;
    tf_msg.child_frame_id = parent_frame;
    tf_msg.transform = tf2::toMsg(parent_T_child.inverse());
  } else {
    tf_msg.header.frame_id = parent_frame;
    tf_msg.child_frame_id = msg->child_frame_id;
    tf_msg.transform = tf2::toMsg(parent_T_child);
  }
  return tf_msg;
}

}  // namespace coug_fg

RCLCPP_COMPONENTS_REGISTER_NODE(coug_fg::OdomToTfNode)
