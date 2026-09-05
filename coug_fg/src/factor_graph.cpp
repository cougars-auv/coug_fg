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

#include "coug_fg/factor_graph.hpp"

#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/Values.h>
#include <rcl/time.h>

#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <diagnostic_updater/diagnostic_status_wrapper.hpp>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <shared_mutex>
#include <stdexcept>
#include <string>
#include <tf2/exceptions.hpp>
#include <tf2/time.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <utility>
#include <vector>

#include "coug_fg/factor_graph_core.hpp"
#include "coug_fg/factor_graph_parameters.hpp"
#include "coug_fg/utils/data_types.hpp"
#include "coug_fg/utils/logger.hpp"
#include "coug_fg/utils/param_enums.hpp"
#include "coug_fg/utils/ros_conversions.hpp"
#include "coug_interfaces/msg/agent_status.hpp"
#include "coug_interfaces/msg/graph_metrics.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace coug_fg {

using coug_interfaces::msg::AgentStatus;
using coug_interfaces::msg::GraphMetrics;

using utils::AgentStatusData;
using utils::AhrsData;
using utils::ImuData;
using utils::KeyframeSource;
using utils::LogLevel;
using utils::MagneticFieldData;
using utils::OdometryData;
using utils::parseKeyframeSource;
using utils::parseSolverType;
using utils::QueueBundle;
using utils::SolverType;
using utils::TfBundle;
using utils::ThreadSafeQueue;
using utils::toCovariance36Msg;
using utils::toCovariance9Msg;
using utils::toGtsam;
using utils::toPoseCovarianceMsg;
using utils::toPoseMsg;
using utils::toQuatMsg;
using utils::toVectorMsg;
using utils::TwistData;
using utils::WrenchData;

namespace {

constexpr double kUnknownCovariance = -1.0;
constexpr size_t kSensorQueueDepth = 200;

template <int N, typename Array>
Eigen::Matrix<double, N, N> toCovMatrix(const Array& arr) {
  return Eigen::Map<const Eigen::Matrix<double, N, N, Eigen::RowMajor>>(arr.data());
}

}  // namespace

void FactorGraphNode::setupRosInterfaces() {
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  global_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(params_.global_odom_topic,
                                                               rclcpp::SystemDefaultsQoS());
  if (params_.publish_smoothed_path) {
    smoothed_path_pub_ = create_publisher<nav_msgs::msg::Path>(params_.smoothed_path_topic,
                                                               rclcpp::SystemDefaultsQoS());
  }
  if (params_.publish_velocity) {
    vel_pub_ = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
        params_.velocity_topic, rclcpp::SystemDefaultsQoS());
  }
  if (params_.publish_imu_bias) {
    imu_bias_pub_ = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
        params_.imu_bias_topic, rclcpp::SystemDefaultsQoS());
  }
  if (params_.publish_mag_bias) {
    mag_bias_pub_ = create_publisher<sensor_msgs::msg::MagneticField>(params_.mag_bias_topic,
                                                                      rclcpp::SystemDefaultsQoS());
  }
  if (params_.publish_graph_metrics) {
    graph_metrics_pub_ =
        create_publisher<GraphMetrics>(params_.graph_metrics_topic, rclcpp::SystemDefaultsQoS());
  }
  if (params_.multiagent.enable_multiagent) {
    multiagent_pubs_.reserve(params_.multiagent_namespaces.size());
    for (const auto& neighbor_ns : params_.multiagent_namespaces) {
      const std::string odom_topic = "/" + neighbor_ns + "/" + params_.multiagent_global_odom_topic;

      multiagent_pubs_.push_back(
          create_publisher<nav_msgs::msg::Odometry>(odom_topic, rclcpp::SystemDefaultsQoS()));
    }
  }

  reset_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  reset_srv_ = create_service<std_srvs::srv::Trigger>(
      params_.reset_service,
      [this](const std_srvs::srv::Trigger::Request::SharedPtr& req,
             const std::shared_ptr<std_srvs::srv::Trigger::Response>& res) {
        resetGraph(req, res);
      },
      rclcpp::ServicesQoS(), reset_cb_group_);

  sensor_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  auto sensor_options = rclcpp::SubscriptionOptions();
  sensor_options.callback_group = sensor_cb_group_;

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      params_.imu_topic, rclcpp::SensorDataQoS().keep_last(kSensorQueueDepth),
      [this](const sensor_msgs::msg::Imu::ConstSharedPtr& msg) { imuCallback(msg); },
      sensor_options);

  if (params_.gps.enable_gps || params_.gps.enable_gps_init_priors) {
    gps_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        params_.gps_odom_topic, rclcpp::SensorDataQoS(),
        [this](const nav_msgs::msg::Odometry::ConstSharedPtr& msg) { gpsCallback(msg); },
        sensor_options);
  }

  if (params_.depth.enable_depth || params_.depth.enable_depth_init_priors) {
    depth_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        params_.depth_odom_topic, rclcpp::SensorDataQoS(),
        [this](const nav_msgs::msg::Odometry::ConstSharedPtr& msg) { depthCallback(msg); },
        sensor_options);
  }

  if (params_.mag.enable_mag) {
    mag_sub_ = create_subscription<sensor_msgs::msg::MagneticField>(
        params_.mag_topic, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::MagneticField::ConstSharedPtr& msg) { magCallback(msg); },
        sensor_options);
  }

  if (params_.ahrs.enable_ahrs || params_.ahrs.enable_ahrs_init_priors ||
      params_.comparison.enable_loose_dvl_preintegration) {
    ahrs_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        params_.ahrs_topic, rclcpp::SensorDataQoS().keep_last(kSensorQueueDepth),
        [this](const sensor_msgs::msg::Imu::ConstSharedPtr& msg) { ahrsCallback(msg); },
        sensor_options);
  }

  if (params_.dvl.enable_dvl || params_.dvl.enable_dvl_init_priors) {
    dvl_sub_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
        params_.dvl_topic, rclcpp::SensorDataQoS(),
        [this](const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr& msg) {
          dvlCallback(msg);
        },
        sensor_options);
  }

  if (params_.wrench.enable_wrench || params_.wrench.enable_wrench_dropout_only) {
    wrench_sub_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
        params_.wrench_topic, rclcpp::SensorDataQoS(),
        [this](const geometry_msgs::msg::WrenchStamped::ConstSharedPtr& msg) {
          wrenchCallback(msg);
        },
        sensor_options);
  }

  if (params_.multiagent.enable_multiagent) {
    multiagent_queues_.reserve(params_.multiagent_namespaces.size());
    multiagent_subs_.reserve(params_.multiagent_namespaces.size());
    for (size_t agent_queue_idx = 0; agent_queue_idx < params_.multiagent_namespaces.size();
         ++agent_queue_idx) {
      const std::string status_topic = "/" + params_.multiagent_namespaces[agent_queue_idx] + "/" +
                                       params_.multiagent_status_topic;

      multiagent_queues_.push_back(
          std::make_unique<ThreadSafeQueue<std::shared_ptr<AgentStatusData>>>());
      const std::function<void(AgentStatus::ConstSharedPtr)> callback =
          [this, agent_queue_idx](const AgentStatus::ConstSharedPtr& msg) {
            multiAgentCallback(msg, agent_queue_idx);
          };
      multiagent_subs_.push_back(create_subscription<AgentStatus>(
          status_topic, rclcpp::SystemDefaultsQoS(), callback, sensor_options));
    }
  }

  if (keyframe_source_ == KeyframeSource::kTimer ||
      backup_keyframe_source_ == KeyframeSource::kTimer) {
    const double period = 1.0 / params_.keyframe_timer_hz;
    keyframe_timer_ =
        create_wall_timer(std::chrono::duration<double>(period), [this]() { notifyFrontend(); });
  }

  if (params_.publish_diagnostics) {
    const std::string ns = this->get_namespace();
    const std::string clean_ns = (ns == "/") ? "" : ns;
    diagnostic_updater_.setHardwareID(clean_ns + "/factor_graph_node");

    const std::string prefix = clean_ns.empty() ? "" : "[" + clean_ns + "] ";

    std::string suffix;
    if (params_.comparison.enable_loose_dvl_preintegration) {
      suffix = " (FL-LPI)";
    } else if (params_.comparison.enable_tight_dvl_preintegration) {
      suffix = " (FL-TPI)";
    } else if (parseSolverType(params_.solver_type) == SolverType::kIsam2) {
      suffix = " (iS2-B)";
    } else {
      suffix = "";
    }

    const std::string sensor_task = prefix + "Sensor Status" + suffix;
    diagnostic_updater_.add(sensor_task, this, &FactorGraphNode::checkSensorStatus);

    const std::string status_task = prefix + "Graph Status" + suffix;
    diagnostic_updater_.add(status_task, this, &FactorGraphNode::checkGraphStatus);
  }
}

void FactorGraphNode::imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr& msg) {
  const std::string child_frame =
      params_.imu.use_parameter_frame ? params_.imu.parameter_frame : msg->header.frame_id;
  if (!loadOrLookupTf(target_T_imu_tf_, child_frame, params_.imu.use_parameter_tf,
                      params_.imu.parameter_tf.position, params_.imu.parameter_tf.orientation)) {
    return;
  }
  {
    const std::scoped_lock lock(tf_mutex_);
    imu_frame_ = child_frame;
  }
  auto imu_msg = std::make_shared<ImuData>();
  imu_msg->timestamp = rclcpp::Time(msg->header.stamp).seconds();
  imu_msg->linear_acceleration = toGtsam(msg->linear_acceleration);
  imu_msg->angular_velocity = toGtsam(msg->angular_velocity);
  imu_msg->linear_acceleration_covariance = toCovMatrix<3>(msg->linear_acceleration_covariance);
  imu_msg->angular_velocity_covariance = toCovMatrix<3>(msg->angular_velocity_covariance);
  imu_queue_.push(imu_msg);
}

void FactorGraphNode::gpsCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& msg) {
  const std::string child_frame =
      params_.gps.use_parameter_frame ? params_.gps.parameter_frame : msg->child_frame_id;
  if (!loadOrLookupTf(target_T_gps_tf_, child_frame, params_.gps.use_parameter_tf,
                      params_.gps.parameter_tf.position, params_.gps.parameter_tf.orientation)) {
    return;
  }
  auto gps_msg = std::make_shared<OdometryData>();
  gps_msg->timestamp = rclcpp::Time(msg->header.stamp).seconds();
  gps_msg->pose = toGtsam(msg->pose.pose);
  gps_msg->pose_covariance = toGtsam(msg->pose.covariance);
  gps_queue_.push(gps_msg);
}

void FactorGraphNode::depthCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& msg) {
  const std::string child_frame =
      params_.depth.use_parameter_frame ? params_.depth.parameter_frame : msg->child_frame_id;
  if (!loadOrLookupTf(target_T_depth_tf_, child_frame, params_.depth.use_parameter_tf,
                      params_.depth.parameter_tf.position,
                      params_.depth.parameter_tf.orientation)) {
    return;
  }
  auto depth_msg = std::make_shared<OdometryData>();
  depth_msg->timestamp = rclcpp::Time(msg->header.stamp).seconds();
  depth_msg->pose = toGtsam(msg->pose.pose);
  depth_msg->pose_covariance = toGtsam(msg->pose.covariance);
  depth_queue_.push(depth_msg);

  if (keyframe_source_ == KeyframeSource::kDepth ||
      backup_keyframe_source_ == KeyframeSource::kDepth) {
    notifyFrontend();
  }
}

void FactorGraphNode::magCallback(const sensor_msgs::msg::MagneticField::ConstSharedPtr& msg) {
  const std::string child_frame =
      params_.mag.use_parameter_frame ? params_.mag.parameter_frame : msg->header.frame_id;
  if (!loadOrLookupTf(target_T_mag_tf_, child_frame, params_.mag.use_parameter_tf,
                      params_.mag.parameter_tf.position, params_.mag.parameter_tf.orientation)) {
    return;
  }
  {
    const std::scoped_lock lock(tf_mutex_);
    mag_frame_ = child_frame;
  }
  auto mag_msg = std::make_shared<MagneticFieldData>();
  mag_msg->timestamp = rclcpp::Time(msg->header.stamp).seconds();
  mag_msg->magnetic_field = toGtsam(msg->magnetic_field);
  mag_msg->magnetic_field_covariance = toCovMatrix<3>(msg->magnetic_field_covariance);
  mag_queue_.push(mag_msg);
}

void FactorGraphNode::ahrsCallback(const sensor_msgs::msg::Imu::ConstSharedPtr& msg) {
  const std::string child_frame =
      params_.ahrs.use_parameter_frame ? params_.ahrs.parameter_frame : msg->header.frame_id;
  if (!loadOrLookupTf(target_T_ahrs_tf_, child_frame, params_.ahrs.use_parameter_tf,
                      params_.ahrs.parameter_tf.position, params_.ahrs.parameter_tf.orientation)) {
    return;
  }
  auto ahrs_msg = std::make_shared<AhrsData>();
  ahrs_msg->timestamp = rclcpp::Time(msg->header.stamp).seconds();
  ahrs_msg->orientation = toGtsam(msg->orientation);
  ahrs_msg->orientation_covariance = toCovMatrix<3>(msg->orientation_covariance);
  ahrs_queue_.push(ahrs_msg);
}

void FactorGraphNode::dvlCallback(
    const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr& msg) {
  const std::string child_frame =
      params_.dvl.use_parameter_frame ? params_.dvl.parameter_frame : msg->header.frame_id;
  if (!loadOrLookupTf(target_T_dvl_tf_, child_frame, params_.dvl.use_parameter_tf,
                      params_.dvl.parameter_tf.position, params_.dvl.parameter_tf.orientation)) {
    return;
  }
  auto dvl_msg = std::make_shared<TwistData>();
  dvl_msg->timestamp = rclcpp::Time(msg->header.stamp).seconds();
  dvl_msg->linear_velocity = toGtsam(msg->twist.twist.linear);
  dvl_msg->velocity_covariance = toGtsam(msg->twist.covariance);
  dvl_queue_.push(dvl_msg);

  if (keyframe_source_ == KeyframeSource::kDvl || backup_keyframe_source_ == KeyframeSource::kDvl) {
    notifyFrontend();
  }
}

void FactorGraphNode::wrenchCallback(const geometry_msgs::msg::WrenchStamped::ConstSharedPtr& msg) {
  const std::string child_frame =
      params_.wrench.use_parameter_frame ? params_.wrench.parameter_frame : msg->header.frame_id;
  if (!loadOrLookupTf(target_T_wrench_tf_, child_frame, params_.wrench.use_parameter_tf,
                      params_.wrench.parameter_tf.position,
                      params_.wrench.parameter_tf.orientation)) {
    return;
  }
  auto wrench_msg = std::make_shared<WrenchData>();
  wrench_msg->timestamp = rclcpp::Time(msg->header.stamp).seconds();
  wrench_msg->force = toGtsam(msg->wrench.force);
  wrench_msg->torque = toGtsam(msg->wrench.torque);
  wrench_queue_.push(wrench_msg);
}

void FactorGraphNode::multiAgentCallback(const AgentStatus::ConstSharedPtr& msg,
                                         size_t agent_queue_idx) {
  const std::string child_frame = params_.multiagent.use_parameter_frame
                                      ? params_.multiagent.parameter_frame
                                      : msg->header.frame_id;
  if (!loadOrLookupTf(target_T_modem_tf_, child_frame, params_.multiagent.use_parameter_tf,
                      params_.multiagent.parameter_tf.position,
                      params_.multiagent.parameter_tf.orientation)) {
    return;
  }
  auto status_msg = std::make_shared<AgentStatusData>();
  status_msg->timestamp = rclcpp::Time(msg->header.stamp).seconds();
  status_msg->pose = toGtsam(msg->local_odometry);
  status_msg->pose_covariance = toGtsam(msg->odometry_covariance);
  status_msg->pressure_depth = msg->pressure_depth;
  status_msg->imu_orientation = toGtsam(msg->imu_orientation);
  status_msg->includes_range = msg->includes_range;
  status_msg->range_dist = msg->range_dist;
  status_msg->includes_usbl = msg->includes_usbl;
  status_msg->usbl_azimuth = msg->usbl_azimuth;
  status_msg->usbl_elevation = msg->usbl_elevation;
  status_msg->includes_position = msg->includes_position;
  status_msg->position_depth = msg->position_depth;
  multiagent_queues_[agent_queue_idx]->push(status_msg);
}

FactorGraphNode::FactorGraphNode(const rclcpp::NodeOptions& options)
    : Node("factor_graph_node", options),
      diagnostic_updater_(this),
      param_listener_(
          std::make_shared<factor_graph_node::ParamListener>(get_node_parameters_interface())),
      params_(param_listener_->get_params()),
      keyframe_source_(parseKeyframeSource(params_.keyframe_source)),
      backup_keyframe_source_(parseKeyframeSource(params_.backup_keyframe_source)) {
  // Ensure the keyframe sources are valid
  auto source_enabled = [this](KeyframeSource source) {
    switch (source) {
      case KeyframeSource::kDvl:
        return params_.dvl.enable_dvl;
      case KeyframeSource::kDepth:
        return params_.depth.enable_depth;
      default:
        return true;
    }
  };
  if (!source_enabled(keyframe_source_) || !source_enabled(backup_keyframe_source_)) {
    RCLCPP_FATAL(get_logger(),
                 "Keyframe source '%s' or backup '%s' references a disabled sensor. Shutting down.",
                 params_.keyframe_source.c_str(), params_.backup_keyframe_source.c_str());
    throw std::runtime_error("Invalid keyframe source configuration.");
  }

  setupRosInterfaces();
  core_ = std::make_unique<FactorGraphCore>(params_);
  core_->setLogCallback([this](LogLevel level, const std::string& msg) {
    switch (level) {
      case LogLevel::kDebug:
        RCLCPP_DEBUG(get_logger(), "%s", msg.c_str());
        break;
      case LogLevel::kInfo:
        RCLCPP_INFO(get_logger(), "%s", msg.c_str());
        break;
      case LogLevel::kWarn:
        RCLCPP_WARN(get_logger(), "%s", msg.c_str());
        break;
      case LogLevel::kError:
        RCLCPP_ERROR(get_logger(), "%s", msg.c_str());
        break;
    }
  });
  frontend_thread_ = std::thread(&FactorGraphNode::frontendThreadLoop, this);
  backend_thread_ = std::thread(&FactorGraphNode::backendThreadLoop, this);

  RCLCPP_INFO(get_logger(), "Initialization complete.");
}

FactorGraphNode::~FactorGraphNode() {
  is_running_.store(false);
  notifyFrontend();
  notifyBackend();
  if (frontend_thread_.joinable()) {
    frontend_thread_.join();
  }
  if (backend_thread_.joinable()) {
    backend_thread_.join();
  }
}

void FactorGraphNode::notifyFrontend() {
  {
    const std::scoped_lock lock(frontend_trigger_mutex_);
    frontend_trigger_ = true;
  }
  frontend_cv_.notify_one();
}

void FactorGraphNode::notifyBackend() {
  {
    const std::scoped_lock lock(backend_trigger_mutex_);
    backend_trigger_ = true;
  }
  backend_cv_.notify_one();
}

bool FactorGraphNode::checkAndUpdateRateLimit(rclcpp::Time& last_time, double max_rate_hz) {
  if (max_rate_hz <= 0.0) {
    return true;
  }
  const rclcpp::Time now = get_clock()->now();
  if (now - last_time < rclcpp::Duration::from_seconds(1.0 / max_rate_hz)) {
    return false;
  }
  last_time = now;
  return true;
}

bool FactorGraphNode::loadOrLookupTf(geometry_msgs::msg::TransformStamped& tf_out,
                                     const std::string& child_frame, bool use_parameter_tf,
                                     const std::vector<double>& position,
                                     const std::vector<double>& orientation) {
  const std::scoped_lock lock(tf_mutex_);
  if (!tf_out.header.frame_id.empty()) {
    return true;
  }

  if (use_parameter_tf) {
    tf_out.header.stamp = get_clock()->now();
    tf_out.header.frame_id = params_.target_frame;
    tf_out.child_frame_id = child_frame;
    tf_out.transform.translation.x = position[0];
    tf_out.transform.translation.y = position[1];
    tf_out.transform.translation.z = position[2];
    tf_out.transform.rotation.x = orientation[0];
    tf_out.transform.rotation.y = orientation[1];
    tf_out.transform.rotation.z = orientation[2];
    tf_out.transform.rotation.w = orientation[3];
  } else {
    try {
      if (!params_.target_frame.empty()) {
        tf_out = tf_buffer_->lookupTransform(params_.target_frame, child_frame, tf2::TimePointZero);
      }
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Could not transform %s to %s: %s",
                           params_.target_frame.c_str(), child_frame.c_str(), ex.what());
    }
  }

  return !tf_out.header.frame_id.empty();
}

TfBundle FactorGraphNode::buildCurrentTfBundle() {
  const std::scoped_lock lock(tf_mutex_);
  TfBundle tfs;

  auto resolve_tf = [](const geometry_msgs::msg::TransformStamped& tf_in, gtsam::Pose3& pose_out) {
    if (!tf_in.header.frame_id.empty()) {
      pose_out = toGtsam(tf_in.transform);
    }
  };

  resolve_tf(target_T_imu_tf_, tfs.target_T_imu);
  resolve_tf(target_T_gps_tf_, tfs.target_T_gps);
  resolve_tf(target_T_depth_tf_, tfs.target_T_depth);
  resolve_tf(target_T_mag_tf_, tfs.target_T_mag);
  resolve_tf(target_T_ahrs_tf_, tfs.target_T_ahrs);
  resolve_tf(target_T_dvl_tf_, tfs.target_T_dvl);
  resolve_tf(target_T_base_tf_, tfs.target_T_base);
  resolve_tf(target_T_wrench_tf_, tfs.target_T_wrench);
  resolve_tf(target_T_modem_tf_, tfs.target_T_modem);
  return tfs;
}

QueueBundle FactorGraphNode::drainAllQueues() {
  QueueBundle queues;
  queues.imu = imu_queue_.drain();
  queues.gps = gps_queue_.drain();
  queues.depth = depth_queue_.drain();
  queues.mag = mag_queue_.drain();
  queues.ahrs = ahrs_queue_.drain();
  queues.dvl = dvl_queue_.drain();
  queues.wrench = wrench_queue_.drain();
  queues.multiagent.resize(multiagent_queues_.size());
  for (size_t agent_queue_idx = 0; agent_queue_idx < multiagent_queues_.size(); ++agent_queue_idx) {
    queues.multiagent[agent_queue_idx] = multiagent_queues_[agent_queue_idx]->drain();
  }
  return queues;
}

void FactorGraphNode::restoreAllQueues(const QueueBundle& queues) {
  imu_queue_.restore(queues.imu);
  gps_queue_.restore(queues.gps);
  depth_queue_.restore(queues.depth);
  mag_queue_.restore(queues.mag);
  ahrs_queue_.restore(queues.ahrs);
  dvl_queue_.restore(queues.dvl);
  wrench_queue_.restore(queues.wrench);
  for (size_t agent_queue_idx = 0;
       agent_queue_idx < multiagent_queues_.size() && agent_queue_idx < queues.multiagent.size();
       ++agent_queue_idx) {
    multiagent_queues_[agent_queue_idx]->restore(queues.multiagent[agent_queue_idx]);
  }
}

void FactorGraphNode::publishGlobalOdom(const gtsam::Pose3& curr_pose,
                                        const gtsam::Matrix& target_pose_cov,
                                        const rclcpp::Time& timestamp) {
  gtsam::Pose3 target_T_base;
  {
    const std::scoped_lock lock(tf_mutex_);
    target_T_base = toGtsam(target_T_base_tf_.transform);
  }
  const gtsam::Pose3 map_T_base = curr_pose * target_T_base;

  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = timestamp;
  odom_msg.header.frame_id = params_.map_frame;
  odom_msg.child_frame_id = params_.base_frame;
  odom_msg.pose.pose = toPoseMsg(map_T_base);

  gtsam::Matrix pose_cov_to_pub = target_pose_cov;

  if (params_.publish_pose_cov) {
    const gtsam::Rot3& map_R_base = map_T_base.rotation();
    gtsam::Matrix66 rot_block = gtsam::Matrix66::Zero();
    rot_block.block<3, 3>(0, 0) = map_R_base.matrix();
    rot_block.block<3, 3>(3, 3) = map_R_base.matrix();

    // Conjugate the target-frame pose covariance into the base-frame tangent space
    const gtsam::Matrix66 base_pose_cov = target_T_base.inverse().AdjointMap() * target_pose_cov *
                                          target_T_base.inverse().AdjointMap().transpose();

    // Conjugate the base-frame tangent covariance onto the map-frame axes
    pose_cov_to_pub = rot_block * base_pose_cov * rot_block.transpose();
  }

  odom_msg.pose.covariance = toPoseCovarianceMsg(gtsam::Matrix66(pose_cov_to_pub));
  odom_msg.twist.covariance[0] = kUnknownCovariance;
  global_odom_pub_->publish(odom_msg);
}

void FactorGraphNode::publishNeighborGlobalOdom(size_t agent_queue_idx,
                                                const gtsam::Pose3& curr_pose,
                                                const gtsam::Matrix& base_pose_cov,
                                                const rclcpp::Time& timestamp) {
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = timestamp;
  odom_msg.header.frame_id = params_.map_frame;
  odom_msg.child_frame_id =
      params_.multiagent_namespaces[agent_queue_idx] + "/" + params_.multiagent_base_frame;
  odom_msg.pose.pose = toPoseMsg(curr_pose);

  gtsam::Matrix pose_cov_to_pub = base_pose_cov;

  if (params_.publish_neighbor_pose_cov) {
    const gtsam::Rot3& map_R_base = curr_pose.rotation();
    gtsam::Matrix66 rot_block = gtsam::Matrix66::Zero();
    rot_block.block<3, 3>(0, 0) = map_R_base.matrix();
    rot_block.block<3, 3>(3, 3) = map_R_base.matrix();

    // Conjugate the base-frame tangent covariance onto the map-frame axes
    pose_cov_to_pub = rot_block * base_pose_cov * rot_block.transpose();
  }

  odom_msg.pose.covariance = toPoseCovarianceMsg(gtsam::Matrix66(pose_cov_to_pub));
  odom_msg.twist.covariance[0] = kUnknownCovariance;
  multiagent_pubs_[agent_queue_idx]->publish(odom_msg);
}

void FactorGraphNode::broadcastGlobalTf(const gtsam::Pose3& curr_pose,
                                        const rclcpp::Time& timestamp) {
  try {
    gtsam::Pose3 target_T_base;
    {
      const std::scoped_lock lock(tf_mutex_);
      target_T_base = toGtsam(target_T_base_tf_.transform);
    }
    const gtsam::Pose3 map_T_base = curr_pose * target_T_base;

    const gtsam::Pose3 odom_T_base = toGtsam(
        tf_buffer_->lookupTransform(params_.odom_frame, params_.base_frame, tf2::TimePointZero)
            .transform);
    const gtsam::Pose3 map_T_odom = map_T_base * odom_T_base.inverse();

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = timestamp;
    tf_msg.header.frame_id = params_.map_frame;
    tf_msg.child_frame_id = params_.odom_frame;
    tf_msg.transform.translation = toVectorMsg(map_T_odom.translation());
    tf_msg.transform.rotation = toQuatMsg(map_T_odom.rotation());
    tf_broadcaster_->sendTransform(tf_msg);
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Could not transform %s to %s: %s",
                         params_.odom_frame.c_str(), params_.base_frame.c_str(), ex.what());
  }
}

void FactorGraphNode::broadcastNeighborGlobalTf(size_t agent_queue_idx,
                                                const gtsam::Pose3& curr_pose,
                                                const rclcpp::Time& timestamp) {
  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = timestamp;
  tf_msg.header.frame_id = params_.map_frame;
  tf_msg.child_frame_id =
      params_.multiagent_namespaces[agent_queue_idx] + "/" + params_.multiagent_base_frame;
  tf_msg.transform.translation = toVectorMsg(curr_pose.translation());
  tf_msg.transform.rotation = toQuatMsg(curr_pose.rotation());
  tf_broadcaster_->sendTransform(tf_msg);
}

void FactorGraphNode::publishSmoothedPath(const gtsam::Values& values,
                                          const rclcpp::Time& timestamp) {
  nav_msgs::msg::Path path_msg;
  path_msg.header.stamp = timestamp;
  path_msg.header.frame_id = params_.map_frame;

  gtsam::Pose3 target_T_base;
  {
    const std::scoped_lock lock(tf_mutex_);
    target_T_base = toGtsam(target_T_base_tf_.transform);
  }

  auto keys_snapshot = core_->snapshotTimeKeys();

  for (const auto& pair : keys_snapshot) {
    if (values.exists(pair.second)) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header.frame_id = params_.map_frame;
      ps.header.stamp = rclcpp::Time(pair.first);
      ps.pose = toPoseMsg(values.at<gtsam::Pose3>(pair.second) * target_T_base);
      path_msg.poses.push_back(ps);
    }
  }
  smoothed_path_pub_->publish(path_msg);
}

void FactorGraphNode::publishVelocity(const gtsam::Vector3& curr_vel, const gtsam::Matrix& vel_cov,
                                      const rclcpp::Time& timestamp) {
  geometry_msgs::msg::TwistWithCovarianceStamped vel_msg;
  vel_msg.header.stamp = timestamp;

  // Velocity of the target frame in the map frame.
  vel_msg.header.frame_id = params_.map_frame;
  vel_msg.twist.twist.linear = toVectorMsg(curr_vel);
  vel_msg.twist.covariance = toCovariance36Msg(gtsam::Matrix33(vel_cov));

  for (int i = 3; i < 6; ++i) {
    vel_msg.twist.covariance[i * 6 + i] = kUnknownCovariance;
  }
  vel_pub_->publish(vel_msg);
}

void FactorGraphNode::publishImuBias(const gtsam::imuBias::ConstantBias& curr_imu_bias,
                                     const gtsam::Matrix& imu_bias_cov,
                                     const rclcpp::Time& timestamp) {
  geometry_msgs::msg::TwistWithCovarianceStamped imu_bias_msg;
  imu_bias_msg.header.stamp = timestamp;
  {
    const std::scoped_lock lock(tf_mutex_);
    imu_bias_msg.header.frame_id = imu_frame_;
  }

  // This maps 'linear' to accelerometer bias and 'angular' to gyroscope bias.
  // Preserves accel/gyro cross-covariance; sensor_msgs/Imu has no field for that.
  imu_bias_msg.twist.twist.linear = toVectorMsg(curr_imu_bias.accelerometer());
  imu_bias_msg.twist.twist.angular = toVectorMsg(curr_imu_bias.gyroscope());
  imu_bias_msg.twist.covariance = toCovariance36Msg(gtsam::Matrix66(imu_bias_cov));

  imu_bias_pub_->publish(imu_bias_msg);
}

void FactorGraphNode::publishMagBias(const gtsam::Point3& curr_mag_bias,
                                     const gtsam::Matrix& mag_bias_cov,
                                     const rclcpp::Time& timestamp) {
  sensor_msgs::msg::MagneticField mag_bias_msg;
  mag_bias_msg.header.stamp = timestamp;
  {
    const std::scoped_lock lock(tf_mutex_);
    mag_bias_msg.header.frame_id = mag_frame_;
  }

  mag_bias_msg.magnetic_field = toVectorMsg(curr_mag_bias);
  mag_bias_msg.magnetic_field_covariance = toCovariance9Msg(gtsam::Matrix33(mag_bias_cov));

  mag_bias_pub_->publish(mag_bias_msg);
}

void FactorGraphNode::publishGraphMetrics(const rclcpp::Time& timestamp) {
  GraphMetrics metrics_msg;
  metrics_msg.header.stamp = timestamp;

  metrics_msg.total_duration = last_total_duration_.load();
  metrics_msg.smoother_duration = last_smoother_duration_.load();
  metrics_msg.cov_duration = last_cov_duration_.load();
  metrics_msg.new_factors = static_cast<uint32_t>(new_factors_.load());
  metrics_msg.total_factors = static_cast<uint32_t>(total_factors_.load());
  metrics_msg.total_variables = static_cast<uint32_t>(total_variables_.load());

  graph_metrics_pub_->publish(metrics_msg);
}

void FactorGraphNode::initializeGraph() {
  if (!loadOrLookupTf(target_T_base_tf_, params_.base_frame, params_.base.use_parameter_tf,
                      params_.base.parameter_tf.position, params_.base.parameter_tf.orientation)) {
    return;
  }

  const QueueBundle init_queues = drainAllQueues();

  if (!core_->initialize(get_clock()->now().seconds(), init_queues, buildCurrentTfBundle())) {
    restoreAllQueues(init_queues);
    return;
  }

  is_initialized_.store(true);
  RCLCPP_INFO(get_logger(), "Graph initialized successfully.");
}

void FactorGraphNode::updateGraph() {
  KeyframeSource active_source = keyframe_source_;
  if (active_source != KeyframeSource::kTimer) {
    std::optional<double> last_received = (active_source == KeyframeSource::kDvl)
                                              ? dvl_queue_.getLastTime()
                                              : depth_queue_.getLastTime();

    std::optional<double> newest_stamp = imu_queue_.getLastTime();
    if (!last_received.has_value() ||
        (newest_stamp.has_value() &&
         (*newest_stamp - *last_received) > params_.keyframe_timeout_sec)) {
      if (backup_keyframe_source_ != KeyframeSource::kNone) {
        active_source = backup_keyframe_source_;
        RCLCPP_WARN(get_logger(), "Primary keyframe source '%s' timed out. Using backup '%s'.",
                    params_.keyframe_source.c_str(), params_.backup_keyframe_source.c_str());
      } else {
        RCLCPP_ERROR(get_logger(),
                     "Primary keyframe source '%s' timed out and no backup is configured. "
                     "No new keyframes will be created.",
                     params_.keyframe_source.c_str());
      }
    }
  }

  std::optional<double> target_time;
  if (active_source == KeyframeSource::kDvl && !dvl_queue_.empty()) {
    target_time = dvl_queue_.getLastTime();
  } else if (active_source == KeyframeSource::kDepth && !depth_queue_.empty()) {
    target_time = depth_queue_.getLastTime();
  } else if (active_source == KeyframeSource::kTimer && !imu_queue_.empty()) {
    target_time = imu_queue_.getLastTime();
  }

  if (!target_time.has_value() ||
      (last_target_time_.has_value() && *target_time <= *last_target_time_)) {
    return;
  }

  if (last_target_time_.has_value() &&
      (*target_time - *last_target_time_) < params_.min_keyframe_interval_sec) {
    RCLCPP_WARN(get_logger(),
                "Keyframe rejected: only %.4f s since the last keyframe (minimum %.4f s).",
                *target_time - *last_target_time_, params_.min_keyframe_interval_sec);
    return;
  }
  last_target_time_ = target_time;

  QueueBundle queues = drainAllQueues();
  auto leftover = core_->update(*target_time, queues, buildCurrentTfBundle());
  restoreAllQueues(leftover ? *leftover : queues);
}

void FactorGraphNode::frontendThreadLoop() {
  while (is_running_.load()) {
    std::unique_lock<std::mutex> lock(frontend_trigger_mutex_);
    frontend_cv_.wait(lock, [this] { return frontend_trigger_ || !is_running_.load(); });
    frontend_trigger_ = false;

    if (!is_running_.load()) {
      break;
    }

    lock.unlock();
    {
      const std::shared_lock reset_lock(reset_mutex_);

      if (has_crashed_.load()) {
        drainAllQueues();
        continue;
      }

      if (!is_initialized_.load()) {
        initializeGraph();
      } else if (checkAndUpdateRateLimit(last_update_time_, params_.max_update_rate_hz)) {
        updateGraph();
        notifyBackend();
      }
    }
  }
}

void FactorGraphNode::optimizeGraph() {
  try {
    auto result = core_->optimize();
    if (!result) {
      return;
    }

    last_total_duration_.store(result->total_duration);
    last_smoother_duration_.store(result->smoother_duration);
    last_cov_duration_.store(result->cov_duration);
    new_factors_.store(result->new_factors);
    total_factors_.store(result->total_factors);
    total_variables_.store(result->total_variables);
    processing_overflow_.store(result->processing_overflow);

    if (result->processing_overflow) {
      RCLCPP_WARN(get_logger(), "Processing overflow. Batching %zu keyframes.",
                  result->new_keyframes);
    }

    static constexpr double kSecondsToNanoseconds = 1e9;
    const rclcpp::Time stamp(static_cast<int64_t>(result->timestamp * kSecondsToNanoseconds));
    publishGlobalOdom(result->pose, result->pose_cov, stamp);
    for (const auto& neighbor : result->neighbor_results) {
      const rclcpp::Time neighbor_stamp(
          static_cast<int64_t>(neighbor.timestamp * kSecondsToNanoseconds));
      publishNeighborGlobalOdom(neighbor.agent_queue_idx, neighbor.pose, neighbor.pose_cov,
                                neighbor_stamp);

      if (params_.publish_neighbor_global_tf) {
        broadcastNeighborGlobalTf(neighbor.agent_queue_idx, neighbor.pose, stamp);
      }
    }

    if (params_.publish_global_tf) {
      broadcastGlobalTf(result->pose, stamp);
    }

    if (params_.publish_smoothed_path) {
      publishSmoothedPath(result->smoothed_path, stamp);
    }

    if (params_.publish_velocity) {
      publishVelocity(result->velocity, result->velocity_cov, stamp);
    }

    if (params_.publish_imu_bias) {
      publishImuBias(result->imu_bias, result->imu_bias_cov, stamp);
    }

    if (params_.publish_mag_bias) {
      publishMagBias(result->mag_bias, result->mag_bias_cov, stamp);
    }

    if (params_.publish_graph_metrics) {
      publishGraphMetrics(stamp);
    }
  } catch (const std::exception& e) {
    RCLCPP_FATAL(get_logger(), "%s", e.what());
    has_crashed_.store(true);
  }
}

void FactorGraphNode::backendThreadLoop() {
  while (is_running_.load()) {
    std::unique_lock<std::mutex> lock(backend_trigger_mutex_);
    backend_cv_.wait(lock, [this] { return backend_trigger_ || !is_running_.load(); });
    backend_trigger_ = false;

    if (!is_running_.load()) {
      break;
    }

    if (is_initialized_.load() && !has_crashed_.load()) {
      lock.unlock();

      const std::shared_lock reset_lock(reset_mutex_);
      if (!is_initialized_.load() || has_crashed_.load()) {
        continue;
      }

      if (checkAndUpdateRateLimit(last_opt_time_, params_.max_opt_rate_hz)) {
        optimizeGraph();
      }
    }
  }
}

void FactorGraphNode::checkSensorStatus(diagnostic_updater::DiagnosticStatusWrapper& stat) {
  bool any_critical_offline = false;
  std::vector<std::string> offline_sensors;

  auto check_queue = [&](const std::string& name, size_t size, std::optional<double> since_arrival,
                         bool enabled, bool is_critical, double timeout) {
    if (!enabled) {
      return;
    }

    const double time_since = since_arrival.value_or(-1.0);

    stat.add(name + " Queue Size", size);
    stat.add(name + " Time Since Last (s)", time_since);

    if (time_since > timeout || (!since_arrival.has_value() && size == 0)) {
      offline_sensors.push_back(name);
      if (is_critical) {
        any_critical_offline = true;
      }
    }
  };

  check_queue("IMU", imu_queue_.size(), imu_queue_.secondsSinceLastArrival(), true, true,
              params_.imu.diagnostic_timeout_sec);
  check_queue("GPS", gps_queue_.size(), gps_queue_.secondsSinceLastArrival(),
              params_.gps.enable_gps, false, params_.gps.diagnostic_timeout_sec);
  check_queue("Depth", depth_queue_.size(), depth_queue_.secondsSinceLastArrival(),
              params_.depth.enable_depth, params_.depth.enable_depth,
              params_.depth.diagnostic_timeout_sec);
  check_queue("Mag", mag_queue_.size(), mag_queue_.secondsSinceLastArrival(),
              params_.mag.enable_mag, false, params_.mag.diagnostic_timeout_sec);
  check_queue("AHRS", ahrs_queue_.size(), ahrs_queue_.secondsSinceLastArrival(),
              params_.ahrs.enable_ahrs, false, params_.ahrs.diagnostic_timeout_sec);
  check_queue("DVL", dvl_queue_.size(), dvl_queue_.secondsSinceLastArrival(),
              params_.dvl.enable_dvl, params_.dvl.enable_dvl, params_.dvl.diagnostic_timeout_sec);
  check_queue("Wrench", wrench_queue_.size(), wrench_queue_.secondsSinceLastArrival(),
              params_.wrench.enable_wrench, false, params_.wrench.diagnostic_timeout_sec);
  for (size_t agent_queue_idx = 0; agent_queue_idx < multiagent_queues_.size(); ++agent_queue_idx) {
    check_queue("Modem (" + params_.multiagent_namespaces[agent_queue_idx] + ")",
                multiagent_queues_[agent_queue_idx]->size(),
                multiagent_queues_[agent_queue_idx]->secondsSinceLastArrival(),
                params_.multiagent.enable_multiagent, false,
                params_.multiagent.diagnostic_timeout_sec);
  }

  if (!offline_sensors.empty()) {
    std::string msg = "Sensor data unavailable: ";
    for (size_t i = 0; i < offline_sensors.size(); ++i) {
      msg += offline_sensors[i];
      if (i < offline_sensors.size() - 1) {
        msg += ", ";
      }
    }
    auto level = any_critical_offline ? diagnostic_msgs::msg::DiagnosticStatus::ERROR
                                      : diagnostic_msgs::msg::DiagnosticStatus::WARN;
    stat.summary(level, msg);
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "All requested sensor data acquired.");
  }
}

void FactorGraphNode::checkGraphStatus(diagnostic_updater::DiagnosticStatusWrapper& stat) {
  if (has_crashed_.load()) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
                 "Optimizer crashed. Waiting for a reset.");
  } else if (!is_initialized_.load()) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Waiting for sensor data.");
  } else if (processing_overflow_.load()) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN,
                 "Processing overflow detected. Batching keyframes.");
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Optimizing factor graph.");
  }
  stat.add("Total Duration (s)", last_total_duration_.load());
  stat.add("Smoother Duration (s)", last_smoother_duration_.load());
  stat.add("Covariance Duration (s)", last_cov_duration_.load());

  stat.add("New Factors", new_factors_.load());
  stat.add("Total Factors", total_factors_.load());
  stat.add("Total Variables", total_variables_.load());
}

void FactorGraphNode::resetGraph(
    const std_srvs::srv::Trigger::Request::SharedPtr&,
    const std::shared_ptr<std_srvs::srv::Trigger::Response>& response) {
  RCLCPP_WARN(get_logger(), "Reset requested.");

  const std::unique_lock reset_lock(reset_mutex_);

  // Discard data and reset estimator state
  drainAllQueues();

  core_ = std::make_unique<FactorGraphCore>(params_);
  core_->setLogCallback([this](LogLevel level, const std::string& msg) {
    switch (level) {
      case LogLevel::kDebug:
        RCLCPP_DEBUG(get_logger(), "%s", msg.c_str());
        break;
      case LogLevel::kInfo:
        RCLCPP_INFO(get_logger(), "%s", msg.c_str());
        break;
      case LogLevel::kWarn:
        RCLCPP_WARN(get_logger(), "%s", msg.c_str());
        break;
      case LogLevel::kError:
        RCLCPP_ERROR(get_logger(), "%s", msg.c_str());
        break;
    }
  });

  is_initialized_.store(false);
  has_crashed_.store(false);
  last_target_time_.reset();
  last_update_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  last_opt_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

  last_total_duration_.store(0.0);
  last_smoother_duration_.store(0.0);
  last_cov_duration_.store(0.0);
  processing_overflow_.store(false);
  new_factors_.store(0);
  total_factors_.store(0);
  total_variables_.store(0);

  RCLCPP_INFO(get_logger(), "Graph reset successfully.");
  response->success = true;
  response->message = "Graph reset successfully.";
}

}  // namespace coug_fg

RCLCPP_COMPONENTS_REGISTER_NODE(coug_fg::FactorGraphNode)
