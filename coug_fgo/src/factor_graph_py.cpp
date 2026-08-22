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

#include "coug_fgo/factor_graph_py.hpp"

#include <gtsam/inference/Symbol.h>

#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <unordered_map>

#include "coug_fgo/utils/ros_conversions.hpp"

using gtsam::symbol_shorthand::B;  // Bias (ax,ay,az,gx,gy,gz)
using gtsam::symbol_shorthand::M;  // Magnetometer hard-iron bias (x,y,z)
using gtsam::symbol_shorthand::V;  // Velocity (x,y,z)

namespace coug_fgo {

namespace {

gtsam::Rot3 toRot3(const Eigen::Vector4d& quat_xyzw) {
  return gtsam::Rot3::Quaternion(quat_xyzw(3), quat_xyzw(0), quat_xyzw(1), quat_xyzw(2));
}

Eigen::Vector4d toQuatXyzw(const gtsam::Rot3& rot) {
  gtsam::Quaternion q = rot.toQuaternion();
  return Eigen::Vector4d(q.x(), q.y(), q.z(), q.w());
}

gtsam::Pose3 toPose3(const Eigen::Vector3d& position, const Eigen::Vector4d& quat_xyzw) {
  return gtsam::Pose3(toRot3(quat_xyzw), gtsam::Point3(position));
}

void pyLogCallback(utils::LogLevel level, const std::string& msg) {
  int py_level = 30;  // logging.WARNING
  switch (level) {
    case utils::LogLevel::kDebug:
      py_level = 10;
      break;
    case utils::LogLevel::kInfo:
      py_level = 20;
      break;
    case utils::LogLevel::kWarn:
      py_level = 30;
      break;
    case utils::LogLevel::kError:
      py_level = 40;
      break;
  }
  pybind11::gil_scoped_acquire gil;
  pybind11::module_::import("logging")
      .attr("getLogger")("coug_fgo.core")
      .attr("log")(py_level, msg);
}

pybind11::dict toStateDict(double time, const gtsam::Pose3& pose,
                           const std::optional<gtsam::Vector3>& velocity,
                           const std::optional<gtsam::imuBias::ConstantBias>& bias,
                           const std::optional<gtsam::Point3>& mag_bias) {
  pybind11::dict d;
  gtsam::Quaternion q = pose.rotation().toQuaternion();

  d["time"] = time;
  d["x"] = pose.translation().x();
  d["y"] = pose.translation().y();
  d["z"] = pose.translation().z();
  d["qx"] = q.x();
  d["qy"] = q.y();
  d["qz"] = q.z();
  d["qw"] = q.w();

  if (velocity) {
    d["vx"] = velocity->x();
    d["vy"] = velocity->y();
    d["vz"] = velocity->z();
  }
  if (bias) {
    d["bias_accel_x"] = bias->accelerometer().x();
    d["bias_accel_y"] = bias->accelerometer().y();
    d["bias_accel_z"] = bias->accelerometer().z();
    d["bias_gyro_x"] = bias->gyroscope().x();
    d["bias_gyro_y"] = bias->gyroscope().y();
    d["bias_gyro_z"] = bias->gyroscope().z();
  }
  if (mag_bias) {
    d["bias_mag_x"] = mag_bias->x();
    d["bias_mag_y"] = mag_bias->y();
    d["bias_mag_z"] = mag_bias->z();
  }
  return d;
}

utils::TfBundle toTfBundle(const FactorGraphPy::TfMap& tfs) {
  static const std::unordered_map<std::string, gtsam::Pose3 utils::TfBundle::*> kTfFields = {
      {"imu", &utils::TfBundle::target_T_imu},     {"gps", &utils::TfBundle::target_T_gps},
      {"depth", &utils::TfBundle::target_T_depth}, {"mag", &utils::TfBundle::target_T_mag},
      {"ahrs", &utils::TfBundle::target_T_ahrs},   {"dvl", &utils::TfBundle::target_T_dvl},
      {"base", &utils::TfBundle::target_T_base},   {"com", &utils::TfBundle::target_T_com},
      {"modem", &utils::TfBundle::target_T_modem}};

  utils::TfBundle bundle;
  for (const auto& [name, tf] : tfs) {
    auto it = kTfFields.find(name);
    if (it == kTfFields.end()) {
      throw std::invalid_argument("Unknown transform name: " + name);
    }
    bundle.*(it->second) = toPose3(tf.first, tf.second);
  }
  return bundle;
}

utils::QueueBundle toQueueBundle(
    const FactorGraphPy::ImuBatch& imu, const FactorGraphPy::OdomBatch& gps,
    const FactorGraphPy::DepthBatch& depth, const FactorGraphPy::MagBatch& mag,
    const FactorGraphPy::AhrsBatch& ahrs, const FactorGraphPy::TwistBatch& dvl,
    const FactorGraphPy::WrenchBatch& wrench, const FactorGraphPy::MultiAgentBatch& multiagent) {
  utils::QueueBundle queues;

  for (const auto& [t, accel, gyro, accel_cov, gyro_cov] : imu) {
    auto msg = std::make_shared<utils::ImuData>();
    msg->timestamp = t;
    msg->linear_acceleration = accel;
    msg->angular_velocity = gyro;
    msg->linear_acceleration_covariance = accel_cov;
    msg->angular_velocity_covariance = gyro_cov;
    queues.imu.push_back(msg);
  }

  for (const auto& [t, position, pose_cov] : gps) {
    auto msg = std::make_shared<utils::OdometryData>();
    msg->timestamp = t;
    msg->pose = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(position));
    msg->pose_covariance = utils::swapCovarianceBlocks(pose_cov);
    queues.gps.push_back(msg);
  }

  for (const auto& [t, depth_z, pose_cov] : depth) {
    auto msg = std::make_shared<utils::OdometryData>();
    msg->timestamp = t;
    msg->pose = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 0, depth_z));
    msg->pose_covariance = utils::swapCovarianceBlocks(pose_cov);
    queues.depth.push_back(msg);
  }

  for (const auto& [t, field, field_cov] : mag) {
    auto msg = std::make_shared<utils::MagneticFieldData>();
    msg->timestamp = t;
    msg->magnetic_field = field;
    msg->magnetic_field_covariance = field_cov;
    queues.mag.push_back(msg);
  }

  for (const auto& [t, quat_xyzw, orientation_cov] : ahrs) {
    auto msg = std::make_shared<utils::AhrsData>();
    msg->timestamp = t;
    msg->orientation = toRot3(quat_xyzw);
    msg->orientation_covariance = orientation_cov;
    queues.ahrs.push_back(msg);
  }

  for (const auto& [t, velocity, twist_cov] : dvl) {
    auto msg = std::make_shared<utils::TwistData>();
    msg->timestamp = t;
    msg->linear_velocity = velocity;
    msg->twist_covariance = utils::swapCovarianceBlocks(twist_cov);
    queues.dvl.push_back(msg);
  }

  for (const auto& [t, force_torque] : wrench) {
    auto msg = std::make_shared<utils::WrenchData>();
    msg->timestamp = t;
    msg->force = force_torque.head<3>();
    msg->torque = force_torque.tail<3>();
    queues.wrench.push_back(msg);
  }

  queues.multiagent.resize(multiagent.size());
  for (size_t i = 0; i < multiagent.size(); ++i) {
    for (const auto& [t, position, quat_xyzw, pose_cov, depth_z, imu_quat_xyzw, includes_range,
                      range_dist, includes_usbl, usbl_azimuth, usbl_elevation, includes_position,
                      position_depth] : multiagent[i]) {
      auto msg = std::make_shared<utils::AgentStatusData>();
      msg->timestamp = t;
      msg->pose = toPose3(position, quat_xyzw);
      msg->pose_covariance = utils::swapCovarianceBlocks(pose_cov);
      msg->pressure_depth = depth_z;
      msg->imu_orientation = toRot3(imu_quat_xyzw);
      msg->includes_range = includes_range;
      msg->range_dist = range_dist;
      msg->includes_usbl = includes_usbl;
      msg->usbl_azimuth = usbl_azimuth;
      msg->usbl_elevation = usbl_elevation;
      msg->includes_position = includes_position;
      msg->position_depth = position_depth;
      queues.multiagent[i].push_back(msg);
    }
  }

  return queues;
}

pybind11::dict toBatchDict(const utils::QueueBundle& queues) {
  FactorGraphPy::ImuBatch imu;
  for (const auto& m : queues.imu) {
    imu.emplace_back(m->timestamp, m->linear_acceleration, m->angular_velocity,
                     m->linear_acceleration_covariance, m->angular_velocity_covariance);
  }

  FactorGraphPy::OdomBatch gps;
  for (const auto& m : queues.gps) {
    gps.emplace_back(m->timestamp, m->pose.translation(),
                     utils::swapCovarianceBlocks(m->pose_covariance));
  }

  FactorGraphPy::DepthBatch depth;
  for (const auto& m : queues.depth) {
    depth.emplace_back(m->timestamp, m->pose.translation().z(),
                       utils::swapCovarianceBlocks(m->pose_covariance));
  }

  FactorGraphPy::MagBatch mag;
  for (const auto& m : queues.mag) {
    mag.emplace_back(m->timestamp, m->magnetic_field, m->magnetic_field_covariance);
  }

  FactorGraphPy::AhrsBatch ahrs;
  for (const auto& m : queues.ahrs) {
    ahrs.emplace_back(m->timestamp, toQuatXyzw(m->orientation), m->orientation_covariance);
  }

  FactorGraphPy::TwistBatch dvl;
  for (const auto& m : queues.dvl) {
    dvl.emplace_back(m->timestamp, m->linear_velocity,
                     utils::swapCovarianceBlocks(m->twist_covariance));
  }

  FactorGraphPy::WrenchBatch wrench;
  for (const auto& m : queues.wrench) {
    FactorGraphPy::Vector6d force_torque;
    force_torque << m->force, m->torque;
    wrench.emplace_back(m->timestamp, force_torque);
  }

  FactorGraphPy::MultiAgentBatch multiagent;
  multiagent.reserve(queues.multiagent.size());
  for (const auto& agent : queues.multiagent) {
    std::vector<FactorGraphPy::AgentStatus> neighbor;
    for (const auto& m : agent) {
      neighbor.emplace_back(m->timestamp, m->pose.translation(), toQuatXyzw(m->pose.rotation()),
                            utils::swapCovarianceBlocks(m->pose_covariance), m->pressure_depth,
                            toQuatXyzw(m->imu_orientation), m->includes_range, m->range_dist,
                            m->includes_usbl, m->usbl_azimuth, m->usbl_elevation,
                            m->includes_position, m->position_depth);
    }
    multiagent.push_back(std::move(neighbor));
  }

  pybind11::dict batches;
  batches["imu"] = imu;
  batches["gps"] = gps;
  batches["depth"] = depth;
  batches["mag"] = mag;
  batches["ahrs"] = ahrs;
  batches["dvl"] = dvl;
  batches["wrench"] = wrench;
  batches["multiagent"] = multiagent;
  return batches;
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
  pybind11::dict p;

  // --- Node Settings ---
  p["solver_type"] = params_.solver_type;
  p["max_update_rate_hz"] = params_.max_update_rate_hz;
  p["max_opt_rate_hz"] = params_.max_opt_rate_hz;
  p["publish_smoothed_path"] = params_.publish_smoothed_path;
  p["publish_pose_cov"] = params_.publish_pose_cov;
  p["publish_velocity_cov"] = params_.publish_velocity_cov;
  p["publish_imu_bias_cov"] = params_.publish_imu_bias_cov;

  // --- Keyframe Settings ---
  p["keyframe_source"] = params_.keyframe_source;
  p["backup_keyframe_source"] = params_.backup_keyframe_source;
  p["keyframe_timeout_sec"] = params_.keyframe_timeout_sec;
  p["keyframe_timer_hz"] = params_.keyframe_timer_hz;
  p["min_keyframe_interval_sec"] = params_.min_keyframe_interval_sec;

  // --- ROS Topics and Frames ---
  pybind11::dict topics;
  topics["imu"] = params_.imu_topic;
  topics["gps"] = params_.gps_odom_topic;
  topics["depth"] = params_.depth_odom_topic;
  topics["mag"] = params_.mag_topic;
  topics["ahrs"] = params_.ahrs_topic;
  topics["dvl"] = params_.dvl_topic;
  topics["wrench"] = params_.wrench_topic;
  p["topics"] = topics;

  p["target_frame"] = params_.target_frame;
  p["base_frame"] = params_.base_frame;

  // --- Sensor Settings ---
  auto sensor_dict = [](const auto& s, bool enable, bool enable_init_priors,
                        bool enable_dropout_only = false) {
    pybind11::dict d;
    d["enable"] = enable;
    d["enable_init_priors"] = enable_init_priors;
    d["enable_dropout_only"] = enable_dropout_only;
    d["use_parameter_frame"] = s.use_parameter_frame;
    d["parameter_frame"] = s.parameter_frame;
    d["use_parameter_tf"] = s.use_parameter_tf;
    d["tf_position"] = s.parameter_tf.position;
    d["tf_orientation"] = s.parameter_tf.orientation;
    return d;
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
  sensors["dynamics"] = sensor_dict(params_.dynamics, params_.dynamics.enable_dynamics, false,
                                    params_.dynamics.enable_dynamics_dropout_only);

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
  p["multiagent"] = multiagent;

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
  p["sensors"] = sensors;

  // --- Initial State Priors ---
  pybind11::dict priors;
  priors["use_parameter_priors"] = params_.priors.use_parameter_priors;
  p["priors"] = priors;

  // --- Comparison Methods ---
  pybind11::dict comparison;
  comparison["enable_loose_dvl_preintegration"] =
      params_.comparison.enable_loose_dvl_preintegration;
  comparison["enable_tight_dvl_preintegration"] =
      params_.comparison.enable_tight_dvl_preintegration;
  p["comparison"] = comparison;

  return p;
}

bool FactorGraphPy::initialize(const ImuBatch& imu, const OdomBatch& gps, const DepthBatch& depth,
                               const MagBatch& mag, const AhrsBatch& ahrs, const TwistBatch& dvl,
                               const WrenchBatch& wrench, const MultiAgentBatch& multiagent,
                               const TfMap& tfs) {
  if (is_initialized_) {
    return true;
  }

  utils::QueueBundle queues = toQueueBundle(imu, gps, depth, mag, ahrs, dvl, wrench, multiagent);
  is_initialized_ = core_->initialize(queues, toTfBundle(tfs));
  return is_initialized_;
}

pybind11::object FactorGraphPy::update(double target_time, const ImuBatch& imu,
                                       const OdomBatch& gps, const DepthBatch& depth,
                                       const MagBatch& mag, const AhrsBatch& ahrs,
                                       const TwistBatch& dvl, const WrenchBatch& wrench,
                                       const MultiAgentBatch& multiagent, const TfMap& tfs) {
  if (!is_initialized_) {
    return pybind11::none();
  }

  utils::QueueBundle queues = toQueueBundle(imu, gps, depth, mag, ahrs, dvl, wrench, multiagent);
  auto leftover = core_->update(target_time, queues, toTfBundle(tfs));
  if (!leftover) {
    return pybind11::none();
  }
  return toBatchDict(*leftover);
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
  if (params_.publish_velocity_cov) result["vel_cov"] = opt_result->vel_cov;
  if (params_.publish_imu_bias_cov) result["bias_cov"] = opt_result->bias_cov;

  result["processing_overflow"] = opt_result->processing_overflow;
  result["num_keyframes"] = opt_result->num_keyframes;

  if (params_.publish_smoothed_path && !opt_result->all_estimates.empty()) {
    const gtsam::Values& estimates = opt_result->all_estimates;
    pybind11::list smoothed;

    static constexpr double kNanosecondsToSeconds = 1e-9;
    for (const auto& [time_ns, x_key] : core_->snapshotTimeKeys()) {
      if (!estimates.exists(x_key)) {
        continue;
      }
      size_t step = gtsam::Symbol(x_key).index();

      std::optional<gtsam::Vector3> velocity;
      if (estimates.exists(V(step))) {
        velocity = estimates.at<gtsam::Vector3>(V(step));
      }
      std::optional<gtsam::imuBias::ConstantBias> bias;
      if (estimates.exists(B(step))) {
        bias = estimates.at<gtsam::imuBias::ConstantBias>(B(step));
      }
      std::optional<gtsam::Point3> step_mag_bias;
      if (estimates.exists(M(0))) {
        step_mag_bias = estimates.at<gtsam::Point3>(M(0));
      }
      smoothed.append(toStateDict(static_cast<double>(time_ns) * kNanosecondsToSeconds,
                                  estimates.at<gtsam::Pose3>(x_key), velocity, bias,
                                  step_mag_bias));
    }
    result["smoothed_path"] = smoothed;
  }
  return result;
}

}  // namespace coug_fgo

using coug_fgo::FactorGraphPy;

PYBIND11_MODULE(coug_fgo_py, m) {
  m.doc() = "Python bindings for the FactorGraphCore.";

  pybind11::class_<FactorGraphPy>(m, "FactorGraphPy")
      .def(pybind11::init<const std::vector<std::string>&, const std::string&>(),
           pybind11::arg("config_paths"), pybind11::arg("namespace") = "")
      .def("get_params", &FactorGraphPy::get_params)
      .def("initialize", &FactorGraphPy::initialize,
           pybind11::arg("imu") = FactorGraphPy::ImuBatch(),
           pybind11::arg("gps") = FactorGraphPy::OdomBatch(),
           pybind11::arg("depth") = FactorGraphPy::DepthBatch(),
           pybind11::arg("mag") = FactorGraphPy::MagBatch(),
           pybind11::arg("ahrs") = FactorGraphPy::AhrsBatch(),
           pybind11::arg("dvl") = FactorGraphPy::TwistBatch(),
           pybind11::arg("wrench") = FactorGraphPy::WrenchBatch(),
           pybind11::arg("multiagent") = FactorGraphPy::MultiAgentBatch(), pybind11::kw_only(),
           pybind11::arg("tfs"))
      .def("update", &FactorGraphPy::update, pybind11::arg("target_time"),
           pybind11::arg("imu") = FactorGraphPy::ImuBatch(),
           pybind11::arg("gps") = FactorGraphPy::OdomBatch(),
           pybind11::arg("depth") = FactorGraphPy::DepthBatch(),
           pybind11::arg("mag") = FactorGraphPy::MagBatch(),
           pybind11::arg("ahrs") = FactorGraphPy::AhrsBatch(),
           pybind11::arg("dvl") = FactorGraphPy::TwistBatch(),
           pybind11::arg("wrench") = FactorGraphPy::WrenchBatch(),
           pybind11::arg("multiagent") = FactorGraphPy::MultiAgentBatch(), pybind11::kw_only(),
           pybind11::arg("tfs"))
      .def("optimize", &FactorGraphPy::optimize);
}
