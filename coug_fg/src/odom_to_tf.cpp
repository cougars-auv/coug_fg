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

#include "coug_fg/odom_to_tf.hpp"

#include <functional>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp_components/register_node_macro.hpp>
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
      [this](nav_msgs::msg::Odometry::SharedPtr msg) { odomCallback(msg); });

  RCLCPP_INFO(get_logger(), "Initialization complete.");
}

void OdomToTfNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr& msg) {
  tf_broadcaster_->sendTransform(convertToTf(msg));
}

auto OdomToTfNode::convertToTf(const nav_msgs::msg::Odometry::SharedPtr& msg)
    -> geometry_msgs::msg::TransformStamped {
  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header = msg->header;
  tf_msg.child_frame_id = msg->child_frame_id;
  tf_msg.transform.translation.x = msg->pose.pose.position.x;
  tf_msg.transform.translation.y = msg->pose.pose.position.y;
  tf_msg.transform.translation.z = msg->pose.pose.position.z;
  tf_msg.transform.rotation = msg->pose.pose.orientation;
  return tf_msg;
}

}  // namespace coug_fg

RCLCPP_COMPONENTS_REGISTER_NODE(coug_fg::OdomToTfNode)
