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

#include "coug_fgo/odom_to_tf.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace coug_fgo {

OdomToTfNode::OdomToTfNode(const rclcpp::NodeOptions& options) : Node("odom_to_tf_node", options) {
  param_listener_ =
      std::make_shared<odom_to_tf_node::ParamListener>(get_node_parameters_interface());
  params_ = param_listener_->get_params();

  // --- ROS Interfaces ---
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      params_.input_topic, rclcpp::SensorDataQoS(),
      std::bind(&OdomToTfNode::odomCallback, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "Initialization complete.");
}

void OdomToTfNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  tf_broadcaster_->sendTransform(convertToTf(msg));
}

geometry_msgs::msg::TransformStamped OdomToTfNode::convertToTf(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header = msg->header;
  tf_msg.child_frame_id = msg->child_frame_id;
  tf_msg.transform.translation.x = msg->pose.pose.position.x;
  tf_msg.transform.translation.y = msg->pose.pose.position.y;
  tf_msg.transform.translation.z = msg->pose.pose.position.z;
  tf_msg.transform.rotation = msg->pose.pose.orientation;
  return tf_msg;
}

}  // namespace coug_fgo

RCLCPP_COMPONENTS_REGISTER_NODE(coug_fgo::OdomToTfNode)
