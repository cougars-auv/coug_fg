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

#include "coug_fg/factor_graph_py.hpp"

#include <gtsam/inference/Symbol.h>

#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <unordered_map>

#include "coug_fg/utils/param_enums.hpp"
#include "coug_fg/utils/ros_conversions.hpp"

namespace coug_fg {

using gtsam::symbol_shorthand::B;  // Bias (ax,ay,az,gx,gy,gz)
using gtsam::symbol_shorthand::M;  // Magnetometer hard-iron bias (x,y,z)
using gtsam::symbol_shorthand::V;  // Velocity (x,y,z)

using utils::AgentStatusData;
using utils::AhrsData;
using utils::ImuData;
using utils::LogLevel;
using utils::MagneticFieldData;
using utils::OdometryData;
using utils::QueueBundle;
using utils::swapCovarianceBlocks;
using utils::TfBundle;
using utils::TwistData;
using utils::WrenchData;

namespace {

using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using ImuMsgs = std::vector<
    std::tuple<double, Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d, Eigen::Matrix3d>>;
using GpsMsgs = std::vector<std::tuple<double, Eigen::Vector3d, Matrix6d>>;
using DepthMsgs = std::vector<std::tuple<double, double, Matrix6d>>;
using MagMsgs = std::vector<std::tuple<double, Eigen::Vector3d, Eigen::Matrix3d>>;
using AhrsMsgs = std::vector<std::tuple<double, Eigen::Vector4d, Eigen::Matrix3d>>;
using DvlMsgs = std::vector<std::tuple<double, Eigen::Vector3d, Matrix6d>>;
using WrenchMsgs = std::vector<std::tuple<double, Vector6d>>;
using AgentStatusMsg =
    std::tuple<double, Eigen::Vector3d, Eigen::Vector4d, Matrix6d, double, Eigen::Vector4d, bool,
               double, bool, double, double, bool, double>;
using MultiAgentMsgs = std::vector<std::vector<AgentStatusMsg>>;
using TfMap = std::unordered_map<std::string, std::pair<Eigen::Vector3d, Eigen::Vector4d>>;

gtsam::Rot3 toRot3(const Eigen::Vector4d& q) {
  return gtsam::Rot3::Quaternion(q(3), q(0), q(1), q(2));
}

Eigen::Vector4d toQuatXyzw(const gtsam::Rot3& rot) {
  gtsam::Quaternion q = rot.toQuaternion();
  return Eigen::Vector4d(q.x(), q.y(), q.z(), q.w());
}

gtsam::Pose3 toPose3(const Eigen::Vector3d& position, const Eigen::Vector4d& q) {
  return gtsam::Pose3(toRot3(q), gtsam::Point3(position));
}

void pyLogCallback(LogLevel level, const std::string& msg) {
  int py_level = 30;  // logging.WARNING
  switch (level) {
    case LogLevel::kDebug:
      py_level = 10;
      break;
    case LogLevel::kInfo:
      py_level = 20;
      break;
    case LogLevel::kWarn:
      py_level = 30;
      break;
    case LogLevel::kError:
      py_level = 40;
      break;
  }
  pybind11::gil_scoped_acquire gil;
  pybind11::module_::import("logging").attr("getLogger")("coug_fg.core").attr("log")(py_level, msg);
}

pybind11::dict toStateDict(double time, const gtsam::Pose3& pose,
                           const std::optional<gtsam::Vector3>& velocity,
                           const std::optional<gtsam::imuBias::ConstantBias>& imu_bias,
                           const std::optional<gtsam::Point3>& mag_bias) {
  pybind11::dict state;
  gtsam::Quaternion q = pose.rotation().toQuaternion();

  state["time"] = time;
  state["x"] = pose.translation().x();
  state["y"] = pose.translation().y();
  state["z"] = pose.translation().z();
  state["qx"] = q.x();
  state["qy"] = q.y();
  state["qz"] = q.z();
  state["qw"] = q.w();

  if (velocity) {
    state["vx"] = velocity->x();
    state["vy"] = velocity->y();
    state["vz"] = velocity->z();
  }
  if (imu_bias) {
    state["accel_bias_x"] = imu_bias->accelerometer().x();
    state["accel_bias_y"] = imu_bias->accelerometer().y();
    state["accel_bias_z"] = imu_bias->accelerometer().z();
    state["gyro_bias_x"] = imu_bias->gyroscope().x();
    state["gyro_bias_y"] = imu_bias->gyroscope().y();
    state["gyro_bias_z"] = imu_bias->gyroscope().z();
  }
  if (mag_bias) {
    state["mag_bias_x"] = mag_bias->x();
    state["mag_bias_y"] = mag_bias->y();
    state["mag_bias_z"] = mag_bias->z();
  }
  return state;
}

TfBundle toTfBundle(const TfMap& tfs) {
  static const std::unordered_map<std::string, gtsam::Pose3 TfBundle::*> kTransformFields = {
      {"base", &TfBundle::target_T_base},  {"imu", &TfBundle::target_T_imu},
      {"gps", &TfBundle::target_T_gps},    {"depth", &TfBundle::target_T_depth},
      {"mag", &TfBundle::target_T_mag},    {"ahrs", &TfBundle::target_T_ahrs},
      {"dvl", &TfBundle::target_T_dvl},    {"wrench", &TfBundle::target_T_wrench},
      {"modem", &TfBundle::target_T_modem}};

  TfBundle bundle;
  for (const auto& [name, tf] : tfs) {
    auto it = kTransformFields.find(name);
    if (it == kTransformFields.end()) {
      throw std::invalid_argument("Unknown transform name: " + name);
    }
    bundle.*(it->second) = toPose3(tf.first, tf.second);
  }
  return bundle;
}

QueueBundle toQueueBundle(const ImuMsgs& imu, const GpsMsgs& gps, const DepthMsgs& depth,
                          const MagMsgs& mag, const AhrsMsgs& ahrs, const DvlMsgs& dvl,
                          const WrenchMsgs& wrench, const MultiAgentMsgs& multiagent) {
  QueueBundle queues;

  for (const auto& [t, accel, gyro, accel_cov, gyro_cov] : imu) {
    auto imu_msg = std::make_shared<ImuData>();
    imu_msg->timestamp = t;
    imu_msg->linear_acceleration = accel;
    imu_msg->angular_velocity = gyro;
    imu_msg->linear_acceleration_covariance = accel_cov;
    imu_msg->angular_velocity_covariance = gyro_cov;
    queues.imu.push_back(imu_msg);
  }

  for (const auto& [t, position, pose_cov] : gps) {
    auto gps_msg = std::make_shared<OdometryData>();
    gps_msg->timestamp = t;
    gps_msg->pose = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(position));
    gps_msg->pose_covariance = swapCovarianceBlocks(pose_cov);
    queues.gps.push_back(gps_msg);
  }

  for (const auto& [t, depth_z, pose_cov] : depth) {
    auto depth_msg = std::make_shared<OdometryData>();
    depth_msg->timestamp = t;
    depth_msg->pose = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 0, depth_z));
    depth_msg->pose_covariance = swapCovarianceBlocks(pose_cov);
    queues.depth.push_back(depth_msg);
  }

  for (const auto& [t, field, field_cov] : mag) {
    auto mag_msg = std::make_shared<MagneticFieldData>();
    mag_msg->timestamp = t;
    mag_msg->magnetic_field = field;
    mag_msg->magnetic_field_covariance = field_cov;
    queues.mag.push_back(mag_msg);
  }

  for (const auto& [t, q, orientation_cov] : ahrs) {
    auto ahrs_msg = std::make_shared<AhrsData>();
    ahrs_msg->timestamp = t;
    ahrs_msg->orientation = toRot3(q);
    ahrs_msg->orientation_covariance = orientation_cov;
    queues.ahrs.push_back(ahrs_msg);
  }

  for (const auto& [t, velocity, twist_cov] : dvl) {
    auto dvl_msg = std::make_shared<TwistData>();
    dvl_msg->timestamp = t;
    dvl_msg->linear_velocity = velocity;
    dvl_msg->velocity_covariance = swapCovarianceBlocks(twist_cov);
    queues.dvl.push_back(dvl_msg);
  }

  for (const auto& [t, force_torque] : wrench) {
    auto wrench_msg = std::make_shared<WrenchData>();
    wrench_msg->timestamp = t;
    wrench_msg->force = force_torque.head<3>();
    wrench_msg->torque = force_torque.tail<3>();
    queues.wrench.push_back(wrench_msg);
  }

  queues.multiagent.resize(multiagent.size());
  for (size_t agent_queue_idx = 0; agent_queue_idx < multiagent.size(); ++agent_queue_idx) {
    for (const auto& [t, position, q, pose_cov, depth_z, imu_q, includes_range, range_dist,
                      includes_usbl, usbl_azimuth, usbl_elevation, includes_position,
                      position_depth] : multiagent[agent_queue_idx]) {
      auto status_msg = std::make_shared<AgentStatusData>();
      status_msg->timestamp = t;
      status_msg->pose = toPose3(position, q);
      status_msg->pose_covariance = swapCovarianceBlocks(pose_cov);
      status_msg->pressure_depth = depth_z;
      status_msg->imu_orientation = toRot3(imu_q);
      status_msg->includes_range = includes_range;
      status_msg->range_dist = range_dist;
      status_msg->includes_usbl = includes_usbl;
      status_msg->usbl_azimuth = usbl_azimuth;
      status_msg->usbl_elevation = usbl_elevation;
      status_msg->includes_position = includes_position;
      status_msg->position_depth = position_depth;
      queues.multiagent[agent_queue_idx].push_back(status_msg);
    }
  }

  return queues;
}

template <typename Msgs>
Msgs queueOrEmpty(const pybind11::dict& queues, const char* name) {
  return queues.contains(name) ? queues[name].cast<Msgs>() : Msgs{};
}

QueueBundle toQueueBundle(const pybind11::dict& queues) {
  return toQueueBundle(queueOrEmpty<ImuMsgs>(queues, "imu"), queueOrEmpty<GpsMsgs>(queues, "gps"),
                       queueOrEmpty<DepthMsgs>(queues, "depth"),
                       queueOrEmpty<MagMsgs>(queues, "mag"), queueOrEmpty<AhrsMsgs>(queues, "ahrs"),
                       queueOrEmpty<DvlMsgs>(queues, "dvl"),
                       queueOrEmpty<WrenchMsgs>(queues, "wrench"),
                       queueOrEmpty<MultiAgentMsgs>(queues, "multiagent"));
}

pybind11::dict toQueueDict(const QueueBundle& queue_bundle) {
  ImuMsgs imu;
  for (const auto& imu_msg : queue_bundle.imu) {
    imu.emplace_back(imu_msg->timestamp, imu_msg->linear_acceleration, imu_msg->angular_velocity,
                     imu_msg->linear_acceleration_covariance, imu_msg->angular_velocity_covariance);
  }

  GpsMsgs gps;
  for (const auto& gps_msg : queue_bundle.gps) {
    gps.emplace_back(gps_msg->timestamp, gps_msg->pose.translation(),
                     swapCovarianceBlocks(gps_msg->pose_covariance));
  }

  DepthMsgs depth;
  for (const auto& depth_msg : queue_bundle.depth) {
    depth.emplace_back(depth_msg->timestamp, depth_msg->pose.translation().z(),
                       swapCovarianceBlocks(depth_msg->pose_covariance));
  }

  MagMsgs mag;
  for (const auto& mag_msg : queue_bundle.mag) {
    mag.emplace_back(mag_msg->timestamp, mag_msg->magnetic_field,
                     mag_msg->magnetic_field_covariance);
  }

  AhrsMsgs ahrs;
  for (const auto& ahrs_msg : queue_bundle.ahrs) {
    ahrs.emplace_back(ahrs_msg->timestamp, toQuatXyzw(ahrs_msg->orientation),
                      ahrs_msg->orientation_covariance);
  }

  DvlMsgs dvl;
  for (const auto& dvl_msg : queue_bundle.dvl) {
    dvl.emplace_back(dvl_msg->timestamp, dvl_msg->linear_velocity,
                     swapCovarianceBlocks(dvl_msg->velocity_covariance));
  }

  WrenchMsgs wrench;
  for (const auto& wrench_msg : queue_bundle.wrench) {
    Vector6d force_torque;
    force_torque << wrench_msg->force, wrench_msg->torque;
    wrench.emplace_back(wrench_msg->timestamp, force_torque);
  }

  MultiAgentMsgs multiagent;
  multiagent.reserve(queue_bundle.multiagent.size());
  for (const auto& agent : queue_bundle.multiagent) {
    std::vector<AgentStatusMsg> neighbor;
    for (const auto& status_msg : agent) {
      neighbor.emplace_back(
          status_msg->timestamp, status_msg->pose.translation(),
          toQuatXyzw(status_msg->pose.rotation()),
          swapCovarianceBlocks(status_msg->pose_covariance), status_msg->pressure_depth,
          toQuatXyzw(status_msg->imu_orientation), status_msg->includes_range,
          status_msg->range_dist, status_msg->includes_usbl, status_msg->usbl_azimuth,
          status_msg->usbl_elevation, status_msg->includes_position, status_msg->position_depth);
    }
    multiagent.push_back(std::move(neighbor));
  }

  pybind11::dict queues;
  queues["imu"] = imu;
  queues["gps"] = gps;
  queues["depth"] = depth;
  queues["mag"] = mag;
  queues["ahrs"] = ahrs;
  queues["dvl"] = dvl;
  queues["wrench"] = wrench;
  queues["multiagent"] = multiagent;
  return queues;
}

}  // namespace

FactorGraphPy::FactorGraphPy(const std::vector<std::string>& config_paths, const std::string& ns) {
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  std::vector<std::string> args = {"--ros-args"};
  for (const auto& path : config_paths) {
    args.push_back("--params-file");
    args.push_back(path);
  }

  rclcpp::NodeOptions options;
  options.arguments(args);
  auto param_node = std::make_shared<rclcpp::Node>("factor_graph_node", ns, options);

  factor_graph_node::ParamListener param_listener(param_node->get_node_parameters_interface());
  params_ = param_listener.get_params();

  core_ = std::make_unique<FactorGraphCore>(params_);
  core_->setLogCallback(&pyLogCallback);
}

pybind11::dict FactorGraphPy::get_params() const {
  pybind11::dict params;

  // --- Node Settings ---
  params["solver_type"] = params_.solver_type;
  params["max_update_rate_hz"] = params_.max_update_rate_hz;
  params["max_opt_rate_hz"] = params_.max_opt_rate_hz;
  params["publish_smoothed_path"] = params_.publish_smoothed_path;
  params["publish_pose_cov"] = params_.publish_pose_cov;
  params["publish_velocity_cov"] = params_.publish_velocity_cov;
  params["publish_imu_bias_cov"] = params_.publish_imu_bias_cov;

  // --- Keyframe Settings ---
  params["keyframe_source"] = params_.keyframe_source;
  params["backup_keyframe_source"] = params_.backup_keyframe_source;
  params["keyframe_timeout_sec"] = params_.keyframe_timeout_sec;
  params["keyframe_timer_hz"] = params_.keyframe_timer_hz;
  params["min_keyframe_interval_sec"] = params_.min_keyframe_interval_sec;

  // --- ROS Topics and Frames ---
  pybind11::dict topics;
  topics["imu"] = params_.imu_topic;
  topics["gps"] = params_.gps_odom_topic;
  topics["depth"] = params_.depth_odom_topic;
  topics["mag"] = params_.mag_topic;
  topics["ahrs"] = params_.ahrs_topic;
  topics["dvl"] = params_.dvl_topic;
  topics["wrench"] = params_.wrench_topic;
  params["topics"] = topics;

  params["target_frame"] = params_.target_frame;
  params["base_frame"] = params_.base_frame;

  // --- Sensor Settings ---
  auto sensor_dict = [](const auto& sensor_config, bool enable, bool enable_init_priors,
                        bool enable_dropout_only = false) {
    pybind11::dict sensor;
    sensor["enable"] = enable;
    sensor["enable_init_priors"] = enable_init_priors;
    sensor["enable_dropout_only"] = enable_dropout_only;
    sensor["use_parameter_frame"] = sensor_config.use_parameter_frame;
    sensor["parameter_frame"] = sensor_config.parameter_frame;
    sensor["use_parameter_tf"] = sensor_config.use_parameter_tf;
    sensor["tf_position"] = sensor_config.parameter_tf.position;
    sensor["tf_orientation"] = sensor_config.parameter_tf.orientation;
    return sensor;
  };

  pybind11::dict sensors;
  sensors["imu"] = sensor_dict(params_.imu, true, false);
  sensors["gps"] =
      sensor_dict(params_.gps, params_.gps.enable_gps, params_.gps.enable_gps_init_priors);
  sensors["depth"] = sensor_dict(params_.depth, params_.depth.enable_depth,
                                 params_.depth.enable_depth_init_priors);
  sensors["mag"] = sensor_dict(params_.mag, params_.mag.enable_mag, false);
  sensors["ahrs"] =
      sensor_dict(params_.ahrs, params_.ahrs.enable_ahrs, params_.ahrs.enable_ahrs_init_priors);
  sensors["dvl"] =
      sensor_dict(params_.dvl, params_.dvl.enable_dvl, params_.dvl.enable_dvl_init_priors);
  sensors["wrench"] = sensor_dict(params_.wrench, params_.wrench.enable_wrench, false,
                                  params_.wrench.enable_wrench_dropout_only);

  pybind11::dict multiagent;
  multiagent["enable_multiagent"] = params_.multiagent.enable_multiagent;
  multiagent["namespaces"] = params_.multiagent_namespaces;
  multiagent["status_topic"] = params_.multiagent_status_topic;
  multiagent["global_odom_topic"] = params_.multiagent_global_odom_topic;
  multiagent["use_parameter_frame"] = params_.multiagent.use_parameter_frame;
  multiagent["parameter_frame"] = params_.multiagent.parameter_frame;
  multiagent["use_parameter_tf"] = params_.multiagent.use_parameter_tf;
  multiagent["tf_position"] = params_.multiagent.parameter_tf.position;
  multiagent["tf_orientation"] = params_.multiagent.parameter_tf.orientation;
  params["multiagent"] = multiagent;

  pybind11::dict base;
  base["enable"] = true;
  base["enable_init_priors"] = false;
  base["enable_dropout_only"] = false;
  base["use_parameter_frame"] = false;
  base["parameter_frame"] = params_.base_frame;
  base["use_parameter_tf"] = params_.base.use_parameter_tf;
  base["tf_position"] = params_.base.parameter_tf.position;
  base["tf_orientation"] = params_.base.parameter_tf.orientation;
  sensors["base"] = base;
  params["sensors"] = sensors;

  // --- Initial State Priors ---
  pybind11::dict priors;
  priors["use_parameter_priors"] = params_.priors.use_parameter_priors;
  params["priors"] = priors;

  // --- Comparison Methods ---
  pybind11::dict comparison;
  comparison["enable_loose_dvl_preintegration"] =
      params_.comparison.enable_loose_dvl_preintegration;
  comparison["enable_tight_dvl_preintegration"] =
      params_.comparison.enable_tight_dvl_preintegration;
  params["comparison"] = comparison;

  return params;
}

bool FactorGraphPy::initialize(double init_time, const pybind11::dict& queues,
                               const pybind11::dict& tfs) {
  if (is_initialized_) {
    return true;
  }

  QueueBundle queue_bundle = toQueueBundle(queues);
  is_initialized_ = core_->initialize(init_time, queue_bundle, toTfBundle(tfs.cast<TfMap>()));
  return is_initialized_;
}

pybind11::object FactorGraphPy::update(double target_time, const pybind11::dict& queues,
                                       const pybind11::dict& tfs) {
  if (!is_initialized_) {
    return pybind11::none();
  }

  QueueBundle queue_bundle = toQueueBundle(queues);
  auto leftover = core_->update(target_time, queue_bundle, toTfBundle(tfs.cast<TfMap>()));
  if (!leftover) {
    return pybind11::none();
  }
  return toQueueDict(*leftover);
}

pybind11::dict FactorGraphPy::optimize() {
  if (!is_initialized_) {
    return {};
  }

  auto opt_result = core_->optimize();
  if (!opt_result) {
    return {};
  }

  std::optional<gtsam::Point3> mag_bias;
  if (params_.mag.estimate_hard_iron_bias) {
    mag_bias = opt_result->mag_bias;
  }

  pybind11::dict result = toStateDict(opt_result->timestamp, opt_result->pose, opt_result->velocity,
                                      opt_result->imu_bias, mag_bias);

  if (params_.publish_pose_cov) result["pose_cov"] = opt_result->pose_cov;
  if (params_.publish_velocity_cov) result["velocity_cov"] = opt_result->velocity_cov;
  if (params_.publish_imu_bias_cov) result["imu_bias_cov"] = opt_result->imu_bias_cov;

  result["processing_overflow"] = opt_result->processing_overflow;
  result["new_keyframes"] = opt_result->new_keyframes;

  if (params_.publish_smoothed_path && !opt_result->all_estimates.empty()) {
    const gtsam::Values& estimates = opt_result->all_estimates;
    pybind11::list smoothed;

    static constexpr double kNanosecondsToSeconds = 1e-9;
    for (const auto& [time_ns, x_key] : core_->snapshotTimeKeys()) {
      if (!estimates.exists(x_key)) {
        continue;
      }
      size_t step = gtsam::Symbol(x_key).index();

      std::optional<gtsam::Point3> step_mag_bias;
      if (estimates.exists(M(0))) {
        step_mag_bias = estimates.at<gtsam::Point3>(M(0));
      }
      smoothed.append(
          toStateDict(static_cast<double>(time_ns) * kNanosecondsToSeconds,
                      estimates.at<gtsam::Pose3>(x_key), estimates.at<gtsam::Vector3>(V(step)),
                      estimates.at<gtsam::imuBias::ConstantBias>(B(step)), step_mag_bias));
    }
    result["smoothed_path"] = smoothed;
  }
  return result;
}

}  // namespace coug_fg

PYBIND11_MODULE(coug_fg_py, m) {
  m.doc() = "Python bindings for the FactorGraphCore.";

  using coug_fg::FactorGraphPy;
  using coug_fg::utils::KeyframeSource;
  using coug_fg::utils::parseKeyframeSource;
  using coug_fg::utils::parseRobustKernel;
  using coug_fg::utils::parseSolverType;
  using coug_fg::utils::RobustKernel;
  using coug_fg::utils::SolverType;

  pybind11::enum_<SolverType>(m, "SolverType")
      .value("INCREMENTAL_FIXED_LAG_SMOOTHER", SolverType::kIncrementalFixedLagSmoother)
      .value("ISAM2", SolverType::kIsam2)
      .value("LEVENBERG_MARQUARDT", SolverType::kLevenbergMarquardt);
  m.def("parse_solver_type", &parseSolverType);

  pybind11::enum_<RobustKernel>(m, "RobustKernel")
      .value("NONE", RobustKernel::kNone)
      .value("HUBER", RobustKernel::kHuber)
      .value("TUKEY", RobustKernel::kTukey);
  m.def("parse_robust_kernel", &parseRobustKernel);

  pybind11::enum_<KeyframeSource>(m, "KeyframeSource")
      .value("NONE", KeyframeSource::kNone)
      .value("DVL", KeyframeSource::kDvl)
      .value("DEPTH", KeyframeSource::kDepth)
      .value("TIMER", KeyframeSource::kTimer);
  m.def("parse_keyframe_source", &parseKeyframeSource);

  pybind11::class_<FactorGraphPy>(m, "FactorGraphPy")
      .def(pybind11::init<const std::vector<std::string>&, const std::string&>(),
           pybind11::arg("config_paths"), pybind11::arg("namespace") = "")
      .def("get_params", &FactorGraphPy::get_params)
      .def("initialize", &FactorGraphPy::initialize, pybind11::arg("init_time"),
           pybind11::arg("queues"), pybind11::arg("tfs"))
      .def("update", &FactorGraphPy::update, pybind11::arg("target_time"), pybind11::arg("queues"),
           pybind11::arg("tfs"))
      .def("optimize", &FactorGraphPy::optimize);
}
