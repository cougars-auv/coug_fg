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

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/ImuBias.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <atomic>
#include <condition_variable>
#include <coug_interfaces/msg/agent_status.hpp>
#include <coug_interfaces/msg/graph_metrics.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <memory>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <shared_mutex>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>
#include <vector>

#include "coug_fg/factor_graph_core.hpp"
#include "coug_fg/factor_graph_parameters.hpp"
#include "coug_fg/utils/param_enums.hpp"
#include "coug_fg/utils/ros_conversions.hpp"
#include "coug_fg/utils/thread_safe_queue.hpp"

namespace coug_fg {

class FactorGraphNode : public rclcpp::Node {
 public:
  explicit FactorGraphNode(rclcpp::NodeOptions const& options);

  ~FactorGraphNode() override;

  FactorGraphNode(FactorGraphNode const&) = delete;
  auto operator=(FactorGraphNode const&) -> FactorGraphNode& = delete;
  FactorGraphNode(FactorGraphNode&&) = delete;
  auto operator=(FactorGraphNode&&) -> FactorGraphNode& = delete;

 private:
  // --- Initialization ---
  void setupRosInterfaces();

  // --- Main Logic ---
  void initializeGraph();

  void updateGraph();

  void optimizeGraph();

  void resetGraph(std_srvs::srv::Trigger::Request::SharedPtr const& request,
                  std::shared_ptr<std_srvs::srv::Trigger::Response> const& response);

  // --- Sensor Callbacks ---
  void imuCallback(sensor_msgs::msg::Imu::ConstSharedPtr const& msg);

  void gpsCallback(nav_msgs::msg::Odometry::ConstSharedPtr const& msg);

  void depthCallback(nav_msgs::msg::Odometry::ConstSharedPtr const& msg);

  void magCallback(sensor_msgs::msg::MagneticField::ConstSharedPtr const& msg);

  void ahrsCallback(sensor_msgs::msg::Imu::ConstSharedPtr const& msg);

  void dvlCallback(geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr const& msg);

  void wrenchCallback(geometry_msgs::msg::WrenchStamped::ConstSharedPtr const& msg);

  void multiAgentCallback(coug_interfaces::msg::AgentStatus::ConstSharedPtr const& msg,
                          size_t agent_queue_idx);

  void frontendThreadLoop();

  void backendThreadLoop();

  // --- Helpers ---
  void notifyFrontend();

  void notifyBackend();

  auto checkAndUpdateRateLimit(rclcpp::Time& last_time, double max_rate_hz) -> bool;

  auto loadOrLookupTf(geometry_msgs::msg::TransformStamped& tf_out, std::string const& child_frame,
                      bool use_parameter_tf, std::vector<double> const& position,
                      std::vector<double> const& orientation) -> bool;

  auto buildCurrentTfBundle() -> utils::TfBundle;

  auto drainAllQueues() -> utils::QueueBundle;

  void restoreAllQueues(utils::QueueBundle const& queues);

  // --- Publishing ---
  void publishGlobalOdom(gtsam::Pose3 const& curr_pose, gtsam::Matrix const& target_pose_cov,
                         rclcpp::Time const& timestamp);

  void publishNeighborGlobalOdom(size_t agent_queue_idx, gtsam::Pose3 const& curr_pose,
                                 gtsam::Matrix const& base_pose_cov, rclcpp::Time const& timestamp);

  void broadcastGlobalTf(gtsam::Pose3 const& curr_pose, rclcpp::Time const& timestamp);

  void broadcastNeighborGlobalTf(size_t agent_queue_idx, gtsam::Pose3 const& curr_pose,
                                 rclcpp::Time const& timestamp);

  void publishSmoothedPath(gtsam::Values const& values, rclcpp::Time const& timestamp);

  void publishVelocity(gtsam::Vector3 const& curr_vel, gtsam::Matrix const& vel_cov,
                       rclcpp::Time const& timestamp);

  void publishImuBias(gtsam::imuBias::ConstantBias const& curr_imu_bias,
                      gtsam::Matrix const& imu_bias_cov, rclcpp::Time const& timestamp);

  void publishMagBias(gtsam::Point3 const& curr_mag_bias, gtsam::Matrix const& mag_bias_cov,
                      rclcpp::Time const& timestamp);

  void publishGraphMetrics(rclcpp::Time const& timestamp);

  // --- Diagnostics ---
  void checkSensorStatus(diagnostic_updater::DiagnosticStatusWrapper& stat);

  void checkGraphStatus(diagnostic_updater::DiagnosticStatusWrapper& stat);

  // --- ROS Interfaces ---
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr global_odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr smoothed_path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr imu_bias_pub_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_bias_pub_;
  rclcpp::Publisher<coug_interfaces::msg::GraphMetrics>::SharedPtr graph_metrics_pub_;
  std::vector<rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr> multiagent_pubs_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gps_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr ahrs_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr dvl_sub_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_;
  std::vector<rclcpp::Subscription<coug_interfaces::msg::AgentStatus>::SharedPtr> multiagent_subs_;
  rclcpp::TimerBase::SharedPtr keyframe_timer_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  diagnostic_updater::Updater diagnostic_updater_;

  // --- Parameters ---
  std::shared_ptr<factor_graph_node::ParamListener> param_listener_;
  factor_graph_node::Params params_;
  utils::KeyframeSource keyframe_source_{utils::KeyframeSource::kNone};
  utils::KeyframeSource backup_keyframe_source_{utils::KeyframeSource::kNone};

  // --- Core ---
  std::unique_ptr<FactorGraphCore> core_;

  // --- Node State ---
  std::atomic<bool> is_initialized_{false};
  std::atomic<bool> has_crashed_{false};
  rclcpp::Time last_update_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_opt_time_{0, 0, RCL_ROS_TIME};
  std::optional<double> last_target_time_;

  // --- Diagnostics State ---
  std::atomic<double> last_total_duration_{0.0};
  std::atomic<double> last_smoother_duration_{0.0};
  std::atomic<double> last_cov_duration_{0.0};
  std::atomic<bool> processing_overflow_{false};
  std::atomic<size_t> new_factors_{0};
  std::atomic<size_t> total_factors_{0};
  std::atomic<size_t> total_variables_{0};

  // --- Message Queues ---
  utils::ThreadSafeQueue<std::shared_ptr<utils::ImuData>> imu_queue_;
  utils::ThreadSafeQueue<std::shared_ptr<utils::OdometryData>> gps_queue_;
  utils::ThreadSafeQueue<std::shared_ptr<utils::OdometryData>> depth_queue_;
  utils::ThreadSafeQueue<std::shared_ptr<utils::MagneticFieldData>> mag_queue_;
  utils::ThreadSafeQueue<std::shared_ptr<utils::AhrsData>> ahrs_queue_;
  utils::ThreadSafeQueue<std::shared_ptr<utils::TwistData>> dvl_queue_;
  utils::ThreadSafeQueue<std::shared_ptr<utils::WrenchData>> wrench_queue_;
  std::vector<std::unique_ptr<utils::ThreadSafeQueue<std::shared_ptr<utils::AgentStatusData>>>>
      multiagent_queues_;

  // --- Multithreading ---
  std::thread frontend_thread_;
  std::thread backend_thread_;
  std::condition_variable frontend_cv_;
  std::condition_variable backend_cv_;
  std::mutex frontend_trigger_mutex_;
  std::mutex backend_trigger_mutex_;
  std::shared_mutex reset_mutex_;
  bool frontend_trigger_{false};
  bool backend_trigger_{false};
  std::atomic<bool> is_running_{true};

  rclcpp::CallbackGroup::SharedPtr sensor_cb_group_;
  rclcpp::CallbackGroup::SharedPtr reset_cb_group_;

  // --- Transformations ---
  mutable std::mutex tf_mutex_;
  geometry_msgs::msg::TransformStamped target_T_base_tf_;
  geometry_msgs::msg::TransformStamped target_T_dvl_tf_;
  geometry_msgs::msg::TransformStamped target_T_imu_tf_;
  geometry_msgs::msg::TransformStamped target_T_gps_tf_;
  geometry_msgs::msg::TransformStamped target_T_depth_tf_;
  geometry_msgs::msg::TransformStamped target_T_mag_tf_;
  geometry_msgs::msg::TransformStamped target_T_ahrs_tf_;
  geometry_msgs::msg::TransformStamped target_T_wrench_tf_;
  geometry_msgs::msg::TransformStamped target_T_modem_tf_;

  std::string imu_frame_;
  std::string mag_frame_;
};

}  // namespace coug_fg
