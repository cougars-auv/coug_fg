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

#include "coug_fg/factor_graph_core.hpp"

#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iomanip>
#include <optional>
#include <sstream>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "coug_fg/factors/ahrs_factor.hpp"
#include "coug_fg/factors/ahrs_origin_delta_factor.hpp"
#include "coug_fg/factors/ahrs_yaw_factor.hpp"
#include "coug_fg/factors/bearing_factor.hpp"
#include "coug_fg/factors/bearing_origin_delta_factor.hpp"
#include "coug_fg/factors/const_vel_factor.hpp"
#include "coug_fg/factors/depth_factor.hpp"
#include "coug_fg/factors/depth_origin_delta_factor.hpp"
#include "coug_fg/factors/dvl_factor.hpp"
#include "coug_fg/factors/dvl_loose_preint_factor.hpp"
#include "coug_fg/factors/dvl_tight_preint_factor.hpp"
#include "coug_fg/factors/gps_factor.hpp"
#include "coug_fg/factors/mag_calib_factor.hpp"
#include "coug_fg/factors/mag_factor.hpp"
#include "coug_fg/factors/range_factor.hpp"
#include "coug_fg/factors/range_origin_delta_factor.hpp"
#include "coug_fg/factors/wrench_dynamics_factor.hpp"
#include "coug_fg/utils/param_enums.hpp"

using coug_fg::factors::AhrsFactorArm;
using coug_fg::factors::AhrsOriginDeltaFactorArm;
using coug_fg::factors::AhrsYawFactorArm;
using coug_fg::factors::BearingFactorArm;
using coug_fg::factors::BearingOriginDeltaFactorArm;
using coug_fg::factors::ConstVelFactor;
using coug_fg::factors::DepthFactorArm;
using coug_fg::factors::DepthOriginDeltaFactorArm;
using coug_fg::factors::DvlFactorArm;
using coug_fg::factors::DvlLoosePreintFactorArm;
using coug_fg::factors::DvlTightPreintFactorArm;
using coug_fg::factors::Gps2dFactorArm;
using coug_fg::factors::MagCalibFactorArm;
using coug_fg::factors::MagFactorArm;
using coug_fg::factors::RangeFactorArm;
using coug_fg::factors::RangeOriginDeltaFactorArm;
using coug_fg::factors::WrenchDynamicsFactorArm;

using gtsam::symbol_shorthand::B;  // Bias (ax,ay,az,gx,gy,gz)
using gtsam::symbol_shorthand::M;  // Magnetometer hard-iron bias (x,y,z)
using gtsam::symbol_shorthand::N;  // Neighbor agent Pose3 (x,y,z,r,p,y)
using gtsam::symbol_shorthand::O;  // Neighbor origin delta Pose3 (x,y,z,r,p,y)
using gtsam::symbol_shorthand::V;  // Velocity (x,y,z)
using gtsam::symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)

namespace coug_fg {

using utils::DvlLoosePreintegrator;
using utils::DvlTightPreintegrator;
using utils::KeyframeSource;
using utils::parseKeyframeSource;
using utils::parseRobustKernel;
using utils::parseSolverType;
using utils::RobustKernel;
using utils::SolverType;

namespace {

constexpr double kMinIntegrationDt = 1.0e-6;
constexpr double kMinInterpDt = 1.0e-9;
constexpr double kMinRandomWalkDt = 0.001;
constexpr double kSecondsToNanoseconds = 1e9;
constexpr double kInitWaitThrottleSeconds = 5.0;

template <int N = 3>
Eigen::Matrix<double, N, N> sigmasSquaredDiag(const std::vector<double>& sigmas) {
  return Eigen::Matrix<double, N, N>(Eigen::Map<const Eigen::Matrix<double, N, 1>>(sigmas.data())
                                         .array()
                                         .square()
                                         .matrix()
                                         .asDiagonal());
}

void warnCovFallback(const utils::Logger& logger, const std::string& sensor) {
  logger.logOnce(utils::LogLevel::kWarn, "cov_fallback:" + sensor,
                 sensor +
                     " message covariance is unusable (non-finite or non-positive diagonal); "
                     "falling back to the parameter covariance.");
}

template <int N>
Eigen::Matrix<double, N, N> resolveCov(bool use_param, const std::vector<double>& sigmas,
                                       double scalar, const Eigen::Matrix<double, N, N>& msg_cov,
                                       const std::string& sensor, const utils::Logger& logger,
                                       double sensor_unit_scale = 1.0) {
  if (use_param) {
    return sigmasSquaredDiag<N>(sigmas) * scalar;
  }
  if (!msg_cov.allFinite() || (msg_cov.diagonal().array() <= 0.0).any()) {
    warnCovFallback(logger, sensor);
    return sigmasSquaredDiag<N>(sigmas) * scalar;
  }
  return msg_cov * scalar * sensor_unit_scale;
}

double resolveVar(bool use_param, double sigma, double scalar, double msg_var,
                  const std::string& sensor, const utils::Logger& logger) {
  if (use_param) {
    return sigma * sigma * scalar;
  }
  if (!std::isfinite(msg_var) || msg_var <= 0.0) {
    warnCovFallback(logger, sensor);
    return sigma * sigma * scalar;
  }
  return msg_var * scalar;
}

gtsam::SharedNoiseModel applyRobustKernel(const gtsam::SharedNoiseModel& noise,
                                          const std::string& kernel, double k) {
  switch (parseRobustKernel(kernel)) {
    case RobustKernel::kHuber:
      return gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(k),
                                               noise);
    case RobustKernel::kTukey:
      return gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Tukey::Create(k),
                                               noise);
    case RobustKernel::kNone:
      break;
  }
  return noise;
}

gtsam::Rot3 getInterpolatedOrientation(
    const std::deque<std::shared_ptr<utils::AhrsData>>& ahrs_msgs, double target_time) {
  if (ahrs_msgs.empty()) {
    return gtsam::Rot3();
  }

  auto it_after = std::lower_bound(ahrs_msgs.begin(), ahrs_msgs.end(), target_time,
                                   [](const auto& msg, double t) { return msg->timestamp < t; });

  if (it_after == ahrs_msgs.begin()) {
    return ahrs_msgs.front()->orientation;
  }

  // If past the last message, extrapolate into the future
  if (it_after == ahrs_msgs.end()) {
    if (ahrs_msgs.size() < 2) {
      return ahrs_msgs.back()->orientation;
    }
    it_after--;
  }

  double t1 = (*(it_after - 1))->timestamp;
  double t2 = (*it_after)->timestamp;
  double denominator = t2 - t1;

  if (std::abs(denominator) < kMinInterpDt) {
    return (*(it_after - 1))->orientation;
  }

  double alpha = (target_time - t1) / denominator;

  // Use Slerp for quaternion interpolation (handles alpha > 1.0 for extrapolation)
  return (*(it_after - 1))->orientation.slerp(alpha, (*it_after)->orientation);
}

}  // namespace

FactorGraphCore::FactorGraphCore(const factor_graph_node::Params& params) : params_(params) {
  const auto& modem_tf = params_.multiagent.neighbor.modem_tf;
  neighbor_base_T_modem_ =
      gtsam::Pose3(gtsam::Rot3::Quaternion(modem_tf.orientation[3], modem_tf.orientation[0],
                                           modem_tf.orientation[1], modem_tf.orientation[2]),
                   gtsam::Point3(modem_tf.position[0], modem_tf.position[1], modem_tf.position[2]));
}

void FactorGraphCore::setLogCallback(utils::LogCallback callback) {
  logger_.setCallback(std::move(callback));
}

gtsam::Rot3 FactorGraphCore::computeInitialOrientation(
    const std::shared_ptr<utils::AhrsData>& ahrs) const {
  if (ahrs) {
    // Account for AHRS rotation
    gtsam::Rot3 target_R_ahrs = tfs_.target_T_ahrs.rotation();
    gtsam::Rot3 map_R_target_measured = ahrs->orientation * target_R_ahrs.inverse();
    return AhrsFactorArm::trueNorthOrientation(map_R_target_measured,
                                               params_.ahrs.mag_declination_radians);
  }

  double roll = params_.priors.parameter_priors.initial_orientation[0];
  double pitch = params_.priors.parameter_priors.initial_orientation[1];
  double yaw = params_.priors.parameter_priors.initial_orientation[2];

  gtsam::Rot3 base_R_target = tfs_.target_T_base.rotation().inverse();
  return gtsam::Rot3::Ypr(yaw, pitch, roll) * base_R_target;
}

gtsam::Point3 FactorGraphCore::computeInitialPosition(
    const gtsam::Rot3& map_R_target, const std::shared_ptr<utils::OdometryData>& gps,
    const std::shared_ptr<utils::OdometryData>& depth) const {
  gtsam::Point3 map_p_base(params_.priors.parameter_priors.initial_position[0],
                           params_.priors.parameter_priors.initial_position[1],
                           params_.priors.parameter_priors.initial_position[2]);

  gtsam::Point3 target_p_base = tfs_.target_T_base.translation();
  gtsam::Point3 map_p_target = map_p_base - map_R_target.rotate(target_p_base);

  if (gps) {
    // Account for GPS lever arm
    gtsam::Point3 map_p_target_gps = map_R_target.rotate(tfs_.target_T_gps.translation());
    map_p_target = gps->pose.translation() - map_p_target_gps;
  }

  if (depth) {
    // Account for depth lever arm
    gtsam::Point3 map_p_target_depth = map_R_target.rotate(tfs_.target_T_depth.translation());
    map_p_target.z() = depth->pose.translation().z() - map_p_target_depth.z();
  }

  return map_p_target;
}

gtsam::Vector3 FactorGraphCore::computeInitialVelocity(
    const gtsam::Rot3& map_R_target, const std::shared_ptr<utils::TwistData>& dvl) const {
  if (dvl) {
    // Account for DVL rotation
    gtsam::Vector3 target_v_dvl = tfs_.target_T_dvl.rotation().rotate(dvl->linear_velocity);
    return map_R_target.rotate(target_v_dvl);
  }

  gtsam::Vector3 base_v_base =
      Eigen::Map<const Eigen::Vector3d>(params_.priors.parameter_priors.initial_velocity.data());
  gtsam::Vector3 target_v_base = tfs_.target_T_base.rotation().rotate(base_v_base);
  return map_R_target.rotate(target_v_base);
}

gtsam::Matrix6 FactorGraphCore::computeInitialPoseCovariance(
    const gtsam::Rot3& map_R_target, const std::shared_ptr<utils::OdometryData>& gps,
    const std::shared_ptr<utils::OdometryData>& depth,
    const std::shared_ptr<utils::AhrsData>& ahrs) const {
  const auto& sigmas = params_.priors.parameter_priors_covariance;
  gtsam::Matrix3 map_orientation_cov = sigmasSquaredDiag(sigmas.initial_orientation_sigmas);
  gtsam::Matrix3 map_position_cov = sigmasSquaredDiag(sigmas.initial_position_sigmas);

  if (!params_.priors.use_parameter_priors && !params_.priors.use_parameter_priors_covariance) {
    if (ahrs) {
      map_orientation_cov = resolveCov<3>(
          params_.ahrs.use_parameter_covariance,
          params_.ahrs.parameter_covariance.orientation_noise_sigmas,
          params_.ahrs.covariance_scalar, ahrs->orientation_covariance, "AHRS", logger_);
    }
    if (gps) {
      map_position_cov.topLeftCorner<2, 2>() = resolveCov<2>(
          params_.gps.use_parameter_covariance,
          params_.gps.parameter_covariance.position_noise_sigmas, params_.gps.covariance_scalar,
          gps->pose_covariance.block<2, 2>(3, 3), "GPS", logger_);
    }
    if (depth) {
      map_position_cov(2, 2) = resolveVar(params_.depth.use_parameter_covariance,
                                          params_.depth.parameter_covariance.position_z_noise_sigma,
                                          params_.depth.covariance_scalar,
                                          depth->pose_covariance(5, 5), "Depth", logger_);
    }
  }

  // Account for the GPS and depth lever arms
  const gtsam::Point3 target_p_arm_xy =
      gps ? tfs_.target_T_gps.translation() : tfs_.target_T_base.translation();
  const gtsam::Point3 target_p_arm_z =
      depth ? tfs_.target_T_depth.translation() : tfs_.target_T_base.translation();

  gtsam::Matrix3 J_pos_rot = gtsam::skewSymmetric(map_R_target.rotate(target_p_arm_xy));
  J_pos_rot.row(2) = gtsam::skewSymmetric(map_R_target.rotate(target_p_arm_z)).row(2);

  gtsam::Matrix6 map_pose_cov = gtsam::Matrix6::Zero();
  map_pose_cov.topLeftCorner<3, 3>() = map_orientation_cov;
  map_pose_cov.bottomRightCorner<3, 3>() = map_position_cov;

  // Rotate the map-frame pose covariance through the lever arms into the target tangent space
  const gtsam::Matrix3 target_R_map = map_R_target.inverse().matrix();
  gtsam::Matrix6 J_tangent_pose = gtsam::Matrix6::Zero();
  J_tangent_pose.topLeftCorner<3, 3>() = target_R_map;
  J_tangent_pose.bottomLeftCorner<3, 3>() = target_R_map * J_pos_rot;
  J_tangent_pose.bottomRightCorner<3, 3>() = target_R_map;

  return J_tangent_pose * map_pose_cov * J_tangent_pose.transpose();
}

gtsam::Matrix3 FactorGraphCore::computeInitialVelocityCovariance(
    const gtsam::Rot3& map_R_target, const std::shared_ptr<utils::TwistData>& dvl,
    const gtsam::Matrix3& target_orientation_cov) const {
  gtsam::Matrix3 map_velocity_cov;

  if (!params_.priors.use_parameter_priors && !params_.priors.use_parameter_priors_covariance &&
      dvl) {
    // Account for DVL rotation
    const gtsam::Matrix3 map_R_dvl = (map_R_target * tfs_.target_T_dvl.rotation()).matrix();
    const gtsam::Matrix3 dvl_velocity_cov = resolveCov<3>(
        params_.dvl.use_parameter_covariance,
        params_.dvl.parameter_covariance.velocity_noise_sigmas, params_.dvl.covariance_scalar,
        dvl->velocity_covariance.bottomRightCorner<3, 3>(), "DVL", logger_);

    // Conjugate DVL velocity covariance into the map frame
    map_velocity_cov = map_R_dvl * dvl_velocity_cov * map_R_dvl.transpose();
  } else {
    const gtsam::Matrix3 map_R_base = (map_R_target * tfs_.target_T_base.rotation()).matrix();

    // Conjugate prior velocity covariance into the map frame
    map_velocity_cov =
        map_R_base *
        sigmasSquaredDiag(params_.priors.parameter_priors_covariance.initial_velocity_sigmas) *
        map_R_base.transpose();
  }

  const gtsam::Vector3 map_v_target = computeInitialVelocity(map_R_target, dvl);
  const gtsam::Matrix3 J_vel_rot = gtsam::skewSymmetric(map_v_target) * map_R_target.matrix();

  return map_velocity_cov + J_vel_rot * target_orientation_cov * J_vel_rot.transpose();
}

std::optional<FactorGraphCore::InitialState> FactorGraphCore::computeInitialState(
    double init_time, const utils::QueueBundle& queues) const {
  const KeyframeSource kf = parseKeyframeSource(params_.keyframe_source);
  const KeyframeSource backup_kf = parseKeyframeSource(params_.backup_keyframe_source);

  const bool use_param_priors = params_.priors.use_parameter_priors;
  auto prior = [use_param_priors](bool enabled) { return enabled && !use_param_priors; };
  const bool use_gps = prior(params_.gps.enable_gps_init_priors);
  const bool use_depth = prior(params_.depth.enable_depth_init_priors);
  const bool use_ahrs = prior(params_.ahrs.enable_ahrs_init_priors);
  const bool use_dvl = prior(params_.dvl.enable_dvl_init_priors);

  // Additional sensor data needed for init
  const bool need_ahrs = params_.comparison.enable_loose_dvl_preintegration;
  const bool need_dvl =
      params_.dvl.enable_dvl && (params_.comparison.enable_loose_dvl_preintegration ||
                                 params_.comparison.enable_tight_dvl_preintegration);

  auto keyframed_by = [kf, backup_kf](KeyframeSource src) { return kf == src || backup_kf == src; };
  const bool start_depth = params_.depth.enable_depth && keyframed_by(KeyframeSource::kDepth);
  const bool start_dvl = params_.dvl.enable_dvl && keyframed_by(KeyframeSource::kDvl);

  const std::pair<bool, const char*> requirements[] = {
      {queues.imu.empty(), "IMU"},
      {use_gps && queues.gps.empty(), "GPS"},
      {use_depth && queues.depth.empty(), "depth"},
      {(use_ahrs || need_ahrs) && queues.ahrs.empty(), "AHRS"},
      {(use_dvl || need_dvl) && queues.dvl.empty(), "DVL"},
  };
  std::string missing;
  for (const auto& [is_missing, name] : requirements) {
    if (is_missing) {
      missing += missing.empty() ? name : std::string(", ") + name;
    }
  }

  if (!missing.empty()) {
    logger_.logThrottled(utils::LogLevel::kWarn, "init_wait:" + missing, kInitWaitThrottleSeconds,
                         init_time, "Waiting for initialization data: " + missing + ".");
    return std::nullopt;
  }

  auto newest = [](bool take, const auto& msgs) {
    return (take && !msgs.empty()) ? msgs.back() : std::decay_t<decltype(msgs.back())>{};
  };
  auto gps = newest(use_gps, queues.gps);
  auto depth = newest(use_depth, queues.depth);
  auto ahrs = newest(use_ahrs, queues.ahrs);
  auto dvl = newest(use_dvl, queues.dvl);
  auto depth_at_start = newest(start_depth, queues.depth);
  auto dvl_at_start = newest(start_dvl || need_dvl, queues.dvl);
  auto imu = queues.imu.back();

  InitialState state;
  gtsam::Rot3 map_R_target = computeInitialOrientation(ahrs);
  state.pose = gtsam::Pose3(map_R_target, computeInitialPosition(map_R_target, gps, depth));
  state.velocity = computeInitialVelocity(map_R_target, dvl);
  state.imu_bias = gtsam::imuBias::ConstantBias(
      Eigen::Map<const Eigen::Vector3d>(params_.priors.initial_accel_bias.data()),
      Eigen::Map<const Eigen::Vector3d>(params_.priors.initial_gyro_bias.data()));
  state.mag_bias = Eigen::Map<const Eigen::Vector3d>(params_.priors.hard_iron_bias.data());

  state.pose_cov = computeInitialPoseCovariance(map_R_target, gps, depth, ahrs);
  state.velocity_cov =
      computeInitialVelocityCovariance(map_R_target, dvl, state.pose_cov.topLeftCorner<3, 3>());
  state.imu_bias_cov = gtsam::Matrix6::Zero();
  state.imu_bias_cov.topLeftCorner<3, 3>() =
      sigmasSquaredDiag(params_.priors.initial_accel_bias_sigmas);
  state.imu_bias_cov.bottomRightCorner<3, 3>() =
      sigmasSquaredDiag(params_.priors.initial_gyro_bias_sigmas);
  state.mag_bias_cov = sigmasSquaredDiag(params_.priors.hard_iron_bias_sigmas);

  if (kf == KeyframeSource::kDvl && dvl_at_start) {
    state.timestamp = dvl_at_start->timestamp;
  } else if (kf == KeyframeSource::kDepth && depth_at_start) {
    state.timestamp = depth_at_start->timestamp;
  } else {
    state.timestamp = imu->timestamp;
  }

  state.imu = imu;
  state.dvl = dvl_at_start;
  return state;
}

std::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params>
FactorGraphCore::configureImuPreintegration(const InitialState& init_state) const {
  auto imu_params = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU();
  imu_params->n_gravity =
      gtsam::Vector3(params_.imu.gravity[0], params_.imu.gravity[1], params_.imu.gravity[2]);
  imu_params->body_P_sensor = tfs_.target_T_imu;

  // GTSAM preintegration requires continuous-time densities
  const bool use_param_cov = params_.imu.use_parameter_covariance;
  const double imu_dt = 1.0 / params_.imu.sensor_rate_hz;
  imu_params->accelerometerCovariance =
      resolveCov<3>(use_param_cov, params_.imu.parameter_covariance.accel_noise_sigmas,
                    params_.imu.covariance_scalar, init_state.imu->linear_acceleration_covariance,
                    "IMU accelerometer", logger_, imu_dt);
  imu_params->gyroscopeCovariance =
      resolveCov<3>(use_param_cov, params_.imu.parameter_covariance.gyro_noise_sigmas,
                    params_.imu.covariance_scalar, init_state.imu->angular_velocity_covariance,
                    "IMU gyroscope", logger_, imu_dt);
  imu_params->biasAccCovariance = sigmasSquaredDiag(params_.imu.accel_bias_rw_sigmas);
  imu_params->biasOmegaCovariance = sigmasSquaredDiag(params_.imu.gyro_bias_rw_sigmas);
  imu_params->integrationCovariance =
      gtsam::Matrix33::Identity() * params_.imu.integration_covariance;

  return imu_params;
}

void FactorGraphCore::addPriorFactors(const InitialState& init_state,
                                      gtsam::NonlinearFactorGraph& graph, gtsam::Values& values) {
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
      X(0), prev_pose_, gtsam::noiseModel::Gaussian::Covariance(init_state.pose_cov));
  values.insert(X(0), prev_pose_);

  graph.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(
      V(0), prev_vel_, gtsam::noiseModel::Gaussian::Covariance(init_state.velocity_cov));
  values.insert(V(0), prev_vel_);

  graph.emplace_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(
      B(0), prev_imu_bias_, gtsam::noiseModel::Gaussian::Covariance(init_state.imu_bias_cov));
  values.insert(B(0), prev_imu_bias_);

  // Hard-iron bias is static, so one key is shared by every keyframe
  if (params_.mag.estimate_hard_iron_bias) {
    graph.emplace_shared<gtsam::PriorFactor<gtsam::Point3>>(
        M(0), init_state.mag_bias,
        gtsam::noiseModel::Gaussian::Covariance(init_state.mag_bias_cov));
    values.insert(M(0), init_state.mag_bias);
  }

  gtsam::Vector6 prior_pose_sigmas = init_state.pose_cov.diagonal().cwiseSqrt();
  gtsam::Vector3 prior_vel_sigmas = init_state.velocity_cov.diagonal().cwiseSqrt();
  gtsam::Vector6 prior_imu_bias_sigmas = init_state.imu_bias_cov.diagonal().cwiseSqrt();
  gtsam::Vector3 prior_mag_bias_sigmas = init_state.mag_bias_cov.diagonal().cwiseSqrt();

  std::ostringstream oss;
  oss << "Initial state (t=" << std::fixed << std::setprecision(4) << init_state.timestamp
      << "):\n";
  oss << std::scientific;
  oss << "  Position [m]        : " << init_state.pose.translation().transpose() << "\n"
      << "  Orientation [rad]   : " << init_state.pose.rotation().rpy().transpose() << " (RPY)\n"
      << "  Velocity [m/s]      : " << init_state.velocity.transpose() << "\n"
      << "  Accel bias [m/s^2]  : " << init_state.imu_bias.accelerometer().transpose() << "\n"
      << "  Gyro bias [rad/s]   : " << init_state.imu_bias.gyroscope().transpose() << "\n"
      << "  Mag bias [T]        : " << init_state.mag_bias.transpose() << "\n"
      << "Prior sigmas:\n"
      << "  Position [m]        : " << prior_pose_sigmas.tail<3>().transpose() << "\n"
      << "  Orientation [rad]   : " << prior_pose_sigmas.head<3>().transpose() << " (RPY)\n"
      << "  Velocity [m/s]      : " << prior_vel_sigmas.transpose() << "\n"
      << "  Accel bias [m/s^2]  : " << prior_imu_bias_sigmas.head<3>().transpose() << "\n"
      << "  Gyro bias [rad/s]   : " << prior_imu_bias_sigmas.tail<3>().transpose() << "\n"
      << "  Mag bias [T]        : " << prior_mag_bias_sigmas.transpose();
  logger_.log(utils::LogLevel::kInfo, oss.str());
}

bool FactorGraphCore::initialize(double init_time, const utils::QueueBundle& queues,
                                 const utils::TfBundle& tfs) {
  tfs_ = tfs;

  // --- Compute Initial State ---
  std::optional<InitialState> maybe_state = computeInitialState(init_time, queues);
  if (!maybe_state) {
    return false;
  }
  const InitialState& init_state = *maybe_state;

  prev_pose_ = init_state.pose;
  prev_vel_ = init_state.velocity;
  prev_imu_bias_ = init_state.imu_bias;
  prev_mag_bias_ = init_state.mag_bias;
  prev_time_ = init_state.timestamp;

  last_imu_accel_ = init_state.imu->linear_acceleration;
  last_imu_gyro_ = init_state.imu->angular_velocity;

  // --- Build Initial Graph ---
  gtsam::NonlinearFactorGraph initial_graph;
  gtsam::Values initial_values;
  addPriorFactors(init_state, initial_graph, initial_values);

  if (params_.publish_smoothed_path || params_.multiagent.enable_multiagent) {
    time_to_key_[static_cast<int64_t>(prev_time_ * kSecondsToNanoseconds)] = X(0);
  }

  // --- Initialize Preintegrators ---
  imu_preintegrator_ = std::make_unique<gtsam::PreintegratedCombinedMeasurements>(
      configureImuPreintegration(init_state), prev_imu_bias_);

  if (params_.comparison.enable_loose_dvl_preintegration) {
    dvl_loose_preintegrator_ = std::make_unique<DvlLoosePreintegrator>();
    dvl_loose_preintegrator_->reset(prev_pose_.rotation());
  } else if (params_.comparison.enable_tight_dvl_preintegration) {
    dvl_tight_preintegrator_ = std::make_unique<DvlTightPreintegrator>();
    dvl_tight_preintegrator_->reset();
  }

  if (dvl_loose_preintegrator_ || dvl_tight_preintegrator_) {
    if (params_.dvl.enable_dvl) {
      last_dvl_vel_ = init_state.dvl->linear_velocity;
      last_dvl_cov_ = resolveCov<3>(
          params_.dvl.use_parameter_covariance,
          params_.dvl.parameter_covariance.velocity_noise_sigmas, params_.dvl.covariance_scalar,
          init_state.dvl->velocity_covariance.bottomRightCorner<3, 3>(), "DVL", logger_);
    } else {
      last_dvl_vel_ = gtsam::Vector3::Zero();
      last_dvl_cov_ = sigmasSquaredDiag(params_.dvl.parameter_covariance.velocity_noise_sigmas) *
                      params_.dvl.covariance_scalar;
    }
  }

  // --- Initialize Smoother ---
  gtsam::IncrementalFixedLagSmoother::KeyTimestampMap initial_timestamps;
  initial_timestamps[X(0)] = prev_time_;
  initial_timestamps[V(0)] = prev_time_;
  initial_timestamps[B(0)] = prev_time_;
  if (params_.mag.estimate_hard_iron_bias) {
    initial_timestamps[M(0)] = prev_time_;
  }

  gtsam::ISAM2Params isam2_params;
  isam2_params.relinearizeThreshold = params_.relinearize_threshold;
  isam2_params.relinearizeSkip = params_.relinearize_skip;

  switch (parseSolverType(params_.solver_type)) {
    case SolverType::kIsam2:
      isam_ = std::make_unique<gtsam::ISAM2>(isam2_params);
      isam_->update(initial_graph, initial_values);
      break;
    case SolverType::kLevenbergMarquardt:
      lm_graph_ = initial_graph;
      lm_values_ = initial_values;
      break;
    case SolverType::kIncrementalFixedLagSmoother:
      inc_smoother_ = std::make_unique<gtsam::IncrementalFixedLagSmoother>(params_.smoother_lag_sec,
                                                                           isam2_params);
      inc_smoother_->update(initial_graph, initial_values, initial_timestamps);
      break;
  }

  return true;
}

void FactorGraphCore::addGpsFactor(
    gtsam::NonlinearFactorGraph& graph,
    const std::deque<std::shared_ptr<utils::OdometryData>>& gps_msgs) {
  if (gps_msgs.empty()) {
    return;
  }

  const auto& gps_msg = gps_msgs.back();

  gtsam::SharedNoiseModel gps_noise = gtsam::noiseModel::Gaussian::Covariance(resolveCov<2>(
      params_.gps.use_parameter_covariance, params_.gps.parameter_covariance.position_noise_sigmas,
      params_.gps.covariance_scalar, gps_msg->pose_covariance.block<2, 2>(3, 3), "GPS", logger_));

  gps_noise = applyRobustKernel(gps_noise, params_.gps.robust_kernel, params_.gps.robust_k);

  graph.emplace_shared<Gps2dFactorArm>(X(curr_step_), gps_msg->pose.translation(),
                                       tfs_.target_T_gps, gps_noise);
}

void FactorGraphCore::addDepthFactor(
    gtsam::NonlinearFactorGraph& graph,
    const std::deque<std::shared_ptr<utils::OdometryData>>& depth_msgs) {
  if (depth_msgs.empty()) {
    return;
  }

  const auto& depth_msg = depth_msgs.back();

  const double depth_sigma = std::sqrt(resolveVar(
      params_.depth.use_parameter_covariance,
      params_.depth.parameter_covariance.position_z_noise_sigma, params_.depth.covariance_scalar,
      depth_msg->pose_covariance(5, 5), "Depth", logger_));
  gtsam::SharedNoiseModel depth_noise = gtsam::noiseModel::Isotropic::Sigma(1, depth_sigma);

  depth_noise = applyRobustKernel(depth_noise, params_.depth.robust_kernel, params_.depth.robust_k);

  graph.emplace_shared<DepthFactorArm>(X(curr_step_), depth_msg->pose.translation().z(),
                                       tfs_.target_T_depth, depth_noise);
}

void FactorGraphCore::addMagFactor(
    gtsam::NonlinearFactorGraph& graph,
    const std::deque<std::shared_ptr<utils::MagneticFieldData>>& mag_msgs) {
  if (mag_msgs.empty()) {
    return;
  }

  const auto& mag_msg = mag_msgs.back();

  gtsam::Point3 ref_vec(params_.mag.reference_field[0], params_.mag.reference_field[1],
                        params_.mag.reference_field[2]);

  gtsam::SharedNoiseModel mag_noise = gtsam::noiseModel::Gaussian::Covariance(resolveCov<3>(
      params_.mag.use_parameter_covariance,
      params_.mag.parameter_covariance.magnetic_field_noise_sigmas, params_.mag.covariance_scalar,
      mag_msg->magnetic_field_covariance, "Mag", logger_));

  mag_noise = applyRobustKernel(mag_noise, params_.mag.robust_kernel, params_.mag.robust_k);

  if (params_.mag.estimate_hard_iron_bias) {
    graph.emplace_shared<MagCalibFactorArm>(X(curr_step_), M(0), mag_msg->magnetic_field, ref_vec,
                                            tfs_.target_T_mag, mag_noise);
    return;
  }

  // Apply the configured hard-iron bias directly when it is not being estimated
  graph.emplace_shared<MagFactorArm>(X(curr_step_), mag_msg->magnetic_field - prev_mag_bias_,
                                     ref_vec, tfs_.target_T_mag, mag_noise);
}

void FactorGraphCore::addAhrsFactor(gtsam::NonlinearFactorGraph& graph,
                                    const std::deque<std::shared_ptr<utils::AhrsData>>& ahrs_msgs) {
  if (ahrs_msgs.empty()) {
    return;
  }

  const auto& ahrs_msg = ahrs_msgs.back();

  if (params_.ahrs.constrain_yaw_only) {
    const double ahrs_yaw_var = resolveVar(
        params_.ahrs.use_parameter_covariance,
        params_.ahrs.parameter_covariance.orientation_noise_sigmas[2],
        params_.ahrs.covariance_scalar, ahrs_msg->orientation_covariance(2, 2), "AHRS", logger_);
    gtsam::SharedNoiseModel ahrs_noise =
        gtsam::noiseModel::Isotropic::Sigma(1, std::sqrt(ahrs_yaw_var));

    ahrs_noise = applyRobustKernel(ahrs_noise, params_.ahrs.robust_kernel, params_.ahrs.robust_k);

    graph.emplace_shared<AhrsYawFactorArm>(X(curr_step_), ahrs_msg->orientation, tfs_.target_T_ahrs,
                                           params_.ahrs.mag_declination_radians, ahrs_noise);
    return;
  }

  const gtsam::Matrix3 map_ahrs_cov = resolveCov<3>(
      params_.ahrs.use_parameter_covariance,
      params_.ahrs.parameter_covariance.orientation_noise_sigmas, params_.ahrs.covariance_scalar,
      ahrs_msg->orientation_covariance, "AHRS", logger_);

  // Conjugate map-frame orientation covariance into the sensor-frame tangent space
  gtsam::SharedNoiseModel ahrs_noise = gtsam::noiseModel::Gaussian::Covariance(
      AhrsFactorArm::sensorTangentCovariance(map_ahrs_cov, ahrs_msg->orientation));

  ahrs_noise = applyRobustKernel(ahrs_noise, params_.ahrs.robust_kernel, params_.ahrs.robust_k);

  graph.emplace_shared<AhrsFactorArm>(X(curr_step_), ahrs_msg->orientation, tfs_.target_T_ahrs,
                                      params_.ahrs.mag_declination_radians, ahrs_noise);
}

void FactorGraphCore::addDvlFactor(gtsam::NonlinearFactorGraph& graph,
                                   const std::deque<std::shared_ptr<utils::TwistData>>& dvl_msgs,
                                   const gtsam::Vector3& imu_gyro) {
  if (dvl_msgs.empty()) {
    return;
  }

  const auto& dvl_msg = dvl_msgs.back();

  gtsam::Matrix3 dvl_velocity_cov = resolveCov<3>(
      params_.dvl.use_parameter_covariance, params_.dvl.parameter_covariance.velocity_noise_sigmas,
      params_.dvl.covariance_scalar, dvl_msg->velocity_covariance.bottomRightCorner<3, 3>(), "DVL",
      logger_);

  // Scale the preintegrator's continuous-time density to the per-sample noise
  const gtsam::Matrix3 gyro_sample_cov =
      imu_preintegrator_->params()->getGyroscopeCovariance() * params_.imu.sensor_rate_hz;

  dvl_velocity_cov +=
      DvlFactorArm::gyroLeverArmCovariance(gyro_sample_cov, tfs_.target_T_dvl, tfs_.target_T_imu);

  gtsam::SharedNoiseModel dvl_noise = gtsam::noiseModel::Gaussian::Covariance(dvl_velocity_cov);

  dvl_noise = applyRobustKernel(dvl_noise, params_.dvl.robust_kernel, params_.dvl.robust_k);

  graph.emplace_shared<DvlFactorArm>(X(curr_step_), V(curr_step_), B(curr_step_), tfs_.target_T_dvl,
                                     tfs_.target_T_imu, dvl_msg->linear_velocity, imu_gyro,
                                     dvl_noise);
}

void FactorGraphCore::addConstVelFactor(gtsam::NonlinearFactorGraph& graph, double target_time) {
  double dt = target_time - prev_time_;
  Eigen::Vector3d vel_random_walk =
      Eigen::Map<const Eigen::Vector3d>(params_.const_vel.prediction_noise_sigmas.data()) *
      std::sqrt(params_.const_vel.covariance_scalar);
  double sqrt_dt = std::sqrt(std::max(dt, kMinRandomWalkDt));
  Eigen::Vector3d scaled_sigma = vel_random_walk * sqrt_dt;

  gtsam::SharedNoiseModel const_vel_noise = gtsam::noiseModel::Diagonal::Sigmas(scaled_sigma);

  const_vel_noise = applyRobustKernel(const_vel_noise, params_.const_vel.robust_kernel,
                                      params_.const_vel.robust_k);

  graph.emplace_shared<ConstVelFactor>(X(prev_step_), V(prev_step_), X(curr_step_), V(curr_step_),
                                       const_vel_noise);
}

void FactorGraphCore::addWrenchDynamicsFactor(
    gtsam::NonlinearFactorGraph& graph,
    const std::deque<std::shared_ptr<utils::WrenchData>>& wrench_msgs, double target_time) {
  // Implements a zero-order hold (ZOH) for wrench commands
  if (!wrench_msgs.empty()) {
    last_wrench_msg_ = wrench_msgs.back();
  }

  if (!last_wrench_msg_) {
    return;
  }

  const auto& wrench_msg = last_wrench_msg_;

  double dt = target_time - prev_time_;
  double sqrt_dt = std::sqrt(std::max(dt, kMinRandomWalkDt));
  gtsam::Vector3 wrench_sigmas =
      Eigen::Map<const Eigen::Vector3d>(params_.wrench.prediction_noise_sigmas.data()) *
      std::sqrt(params_.wrench.covariance_scalar) * sqrt_dt;
  gtsam::SharedNoiseModel wrench_noise = gtsam::noiseModel::Diagonal::Sigmas(wrench_sigmas);

  wrench_noise =
      applyRobustKernel(wrench_noise, params_.wrench.robust_kernel, params_.wrench.robust_k);

  graph.emplace_shared<WrenchDynamicsFactorArm>(
      X(prev_step_), V(prev_step_), X(curr_step_), V(curr_step_), dt, wrench_msg->force,
      tfs_.target_T_wrench,
      gtsam::Matrix33(Eigen::Map<const Eigen::Vector3d>(params_.wrench.mass.data()).asDiagonal()),
      gtsam::Matrix33(
          Eigen::Map<const Eigen::Vector3d>(params_.wrench.linear_drag.data()).asDiagonal()),
      gtsam::Matrix33(
          Eigen::Map<const Eigen::Vector3d>(params_.wrench.quad_drag.data()).asDiagonal()),
      wrench_noise);
}

void FactorGraphCore::addImuPreintFactor(
    gtsam::NonlinearFactorGraph& graph, const std::deque<std::shared_ptr<utils::ImuData>>& imu_msgs,
    double target_time) {
  if (!imu_preintegrator_ || imu_msgs.empty()) {
    return;
  }

  double last_imu_time = prev_time_;

  for (const auto& imu_msg : imu_msgs) {
    double curr_imu_time = imu_msg->timestamp;

    if (curr_imu_time <= last_imu_time) {
      continue;
    }

    double dt = curr_imu_time - last_imu_time;
    if (dt > kMinIntegrationDt) {
      imu_preintegrator_->integrateMeasurement(last_imu_accel_, last_imu_gyro_, dt);
    }

    last_imu_accel_ = imu_msg->linear_acceleration;
    last_imu_gyro_ = imu_msg->angular_velocity;
    last_imu_time = curr_imu_time;
  }

  // Extra measurement to reach exact target time
  if (last_imu_time < target_time) {
    double dt = target_time - last_imu_time;
    if (dt > kMinIntegrationDt) {
      imu_preintegrator_->integrateMeasurement(last_imu_accel_, last_imu_gyro_, dt);
    }
    last_imu_time = target_time;
  }

  graph.emplace_shared<gtsam::CombinedImuFactor>(X(prev_step_), V(prev_step_), X(curr_step_),
                                                 V(curr_step_), B(prev_step_), B(curr_step_),
                                                 *imu_preintegrator_);
}

void FactorGraphCore::addDvlLoosePreintFactor(
    gtsam::NonlinearFactorGraph& graph,
    const std::deque<std::shared_ptr<utils::TwistData>>& dvl_msgs,
    const std::deque<std::shared_ptr<utils::AhrsData>>& ahrs_msgs, double target_time) {
  if (!dvl_loose_preintegrator_ || ahrs_msgs.empty()) {
    return;
  }

  double last_dvl_time = prev_time_;

  gtsam::Rot3 target_R_ahrs = tfs_.target_T_ahrs.rotation();
  gtsam::Rot3 ahrs_R_target = target_R_ahrs.inverse();
  gtsam::Rot3 target_R_dvl = tfs_.target_T_dvl.rotation();

  // Propagate AHRS orientation uncertainty into the preintegrated translation covariance
  const gtsam::Matrix3 map_ahrs_cov = resolveCov<3>(
      params_.ahrs.use_parameter_covariance,
      params_.ahrs.parameter_covariance.orientation_noise_sigmas, params_.ahrs.covariance_scalar,
      ahrs_msgs.back()->orientation_covariance, "AHRS", logger_);

  gtsam::Rot3 map_R_ahrs_prev = getInterpolatedOrientation(ahrs_msgs, prev_time_);

  // Conjugate map-frame orientation covariance into the window-start AHRS-frame tangent space
  const gtsam::Matrix3 ahrs_tangent_cov =
      AhrsFactorArm::sensorTangentCovariance(map_ahrs_cov, map_R_ahrs_prev);
  gtsam::Rot3 map_R_target_prev = map_R_ahrs_prev * ahrs_R_target;
  dvl_loose_preintegrator_->reset(map_R_target_prev, target_R_ahrs, target_R_dvl, ahrs_tangent_cov);

  for (const auto& dvl_msg : dvl_msgs) {
    double curr_dvl_time = dvl_msg->timestamp;
    if (curr_dvl_time <= last_dvl_time) {
      continue;
    }

    double dt = curr_dvl_time - last_dvl_time;
    if (dt > kMinIntegrationDt) {
      // Integrate DVL measurement alongside the interpolated AHRS attitude
      gtsam::Rot3 map_R_ahrs = getInterpolatedOrientation(ahrs_msgs, last_dvl_time);
      gtsam::Rot3 map_R_dvl = map_R_ahrs * ahrs_R_target * target_R_dvl;

      dvl_loose_preintegrator_->integrateMeasurement(last_dvl_vel_, map_R_dvl, dt, last_dvl_cov_);
    }

    last_dvl_vel_ = dvl_msg->linear_velocity;

    last_dvl_cov_ = resolveCov<3>(
        params_.dvl.use_parameter_covariance,
        params_.dvl.parameter_covariance.velocity_noise_sigmas, params_.dvl.covariance_scalar,
        dvl_msg->velocity_covariance.bottomRightCorner<3, 3>(), "DVL", logger_);
    last_dvl_time = curr_dvl_time;
  }

  // Extra measurement to reach exact target time
  if (last_dvl_time < target_time) {
    double dt = target_time - last_dvl_time;
    if (dt > kMinIntegrationDt) {
      gtsam::Rot3 map_R_ahrs = getInterpolatedOrientation(ahrs_msgs, last_dvl_time);
      gtsam::Rot3 map_R_dvl = map_R_ahrs * ahrs_R_target * target_R_dvl;
      dvl_loose_preintegrator_->integrateMeasurement(last_dvl_vel_, map_R_dvl, dt, last_dvl_cov_);
    }
    last_dvl_time = target_time;
  }

  gtsam::SharedNoiseModel preint_noise =
      gtsam::noiseModel::Gaussian::Covariance(dvl_loose_preintegrator_->covariance());

  preint_noise = applyRobustKernel(preint_noise, params_.dvl.robust_kernel, params_.dvl.robust_k);

  graph.emplace_shared<DvlLoosePreintFactorArm>(X(prev_step_), X(curr_step_), tfs_.target_T_dvl,
                                                dvl_loose_preintegrator_->delta(), preint_noise);
}

void FactorGraphCore::addDvlTightPreintFactor(
    gtsam::NonlinearFactorGraph& graph,
    const std::deque<std::shared_ptr<utils::TwistData>>& dvl_msgs,
    const std::deque<std::shared_ptr<utils::ImuData>>& imu_msgs, double target_time,
    const gtsam::Vector3& held_imu_accel, const gtsam::Vector3& held_imu_gyro) {
  if (!dvl_tight_preintegrator_ || imu_msgs.empty()) {
    return;
  }

  double last_dvl_time = prev_time_;
  double last_imu_time = prev_time_;

  gtsam::Rot3 target_R_dvl = tfs_.target_T_dvl.rotation();

  gtsam::PreintegratedCombinedMeasurements temp_imu_preint(*imu_preintegrator_);
  temp_imu_preint.resetIntegrationAndSetBias(prev_imu_bias_);

  gtsam::Vector3 curr_imu_accel = held_imu_accel;
  gtsam::Vector3 curr_imu_gyro = held_imu_gyro;
  auto imu_it = imu_msgs.begin();

  dvl_tight_preintegrator_->reset();

  auto step_imu_preintegrator = [&](double t_end) {
    while (imu_it != imu_msgs.end()) {
      double imu_time = (*imu_it)->timestamp;
      if (imu_time > t_end) {
        break;
      }
      if (imu_time > last_imu_time) {
        double dt_imu = imu_time - last_imu_time;
        temp_imu_preint.integrateMeasurement(curr_imu_accel, curr_imu_gyro, dt_imu);
        last_imu_time = imu_time;
      }
      curr_imu_accel = (*imu_it)->linear_acceleration;
      curr_imu_gyro = (*imu_it)->angular_velocity;
      imu_it++;
    }

    // Extra measurement to reach exact target time
    if (last_imu_time < t_end) {
      double dt_rem = t_end - last_imu_time;
      if (dt_rem > kMinIntegrationDt) {
        temp_imu_preint.integrateMeasurement(curr_imu_accel, curr_imu_gyro, dt_rem);
      }
      last_imu_time = t_end;
    }
  };

  auto sub_imu_it = imu_msgs.begin();

  // Tightly-coupled DVL integration method derived from Thoms et al., IEEE JOE 2023
  auto integrate_dvl_measurement = [&](double t_start, double t_end,
                                       const gtsam::Vector3& curr_dvl_vel) {
    double last_sub_time = t_start;
    while (last_sub_time < t_end) {
      while (sub_imu_it != imu_msgs.end() &&
             (*sub_imu_it)->timestamp <= last_sub_time + kMinIntegrationDt) {
        sub_imu_it++;
      }

      double curr_sub_time = (sub_imu_it != imu_msgs.end() && (*sub_imu_it)->timestamp < t_end)
                                 ? (*sub_imu_it)->timestamp
                                 : t_end;

      double dt = curr_sub_time - last_sub_time;
      if (dt > kMinIntegrationDt) {
        step_imu_preintegrator(last_sub_time);

        gtsam::Rot3 delta_R_ik = temp_imu_preint.deltaRij();
        gtsam::Matrix3 Jr = gtsam::Rot3::ExpmapDerivative(gtsam::Vector3(temp_imu_preint.theta()));
        gtsam::Matrix3 rot_cov_k =
            Jr * temp_imu_preint.preintMeasCov().block<3, 3>(0, 0) * Jr.transpose();
        gtsam::Matrix3 J_bg_k = Jr * temp_imu_preint.preintegrated_H_biasOmega().topRows<3>();

        // Linearly interpolate the DVL velocity onto the IMU timestamp
        double alpha = (last_sub_time - t_start) / (t_end - t_start);
        gtsam::Vector3 interp_dvl_vel = last_dvl_vel_ + alpha * (curr_dvl_vel - last_dvl_vel_);

        gtsam::Matrix3 sub_dvl_cov = last_dvl_cov_ * ((t_end - t_start) / dt);

        dvl_tight_preintegrator_->integrateMeasurement(interp_dvl_vel, delta_R_ik, target_R_dvl, dt,
                                                       sub_dvl_cov, rot_cov_k, J_bg_k);
      }

      last_sub_time = curr_sub_time;
    }
  };

  for (const auto& dvl_msg : dvl_msgs) {
    double curr_dvl_time = dvl_msg->timestamp;
    if (curr_dvl_time <= last_dvl_time) {
      continue;
    }

    double dt = curr_dvl_time - last_dvl_time;
    if (dt > kMinIntegrationDt) {
      integrate_dvl_measurement(last_dvl_time, curr_dvl_time, dvl_msg->linear_velocity);
    }

    last_dvl_vel_ = dvl_msg->linear_velocity;
    last_dvl_cov_ = resolveCov<3>(
        params_.dvl.use_parameter_covariance,
        params_.dvl.parameter_covariance.velocity_noise_sigmas, params_.dvl.covariance_scalar,
        dvl_msg->velocity_covariance.bottomRightCorner<3, 3>(), "DVL", logger_);
    last_dvl_time = curr_dvl_time;
  }

  // Extra measurement to reach exact target time
  if (last_dvl_time < target_time) {
    double dt = target_time - last_dvl_time;
    if (dt > kMinIntegrationDt) {
      integrate_dvl_measurement(last_dvl_time, target_time, last_dvl_vel_);
    }
    last_dvl_time = target_time;
  }

  gtsam::SharedNoiseModel preint_noise =
      gtsam::noiseModel::Gaussian::Covariance(dvl_tight_preintegrator_->covariance());

  preint_noise = applyRobustKernel(preint_noise, params_.dvl.robust_kernel, params_.dvl.robust_k);

  graph.emplace_shared<DvlTightPreintFactorArm>(
      X(prev_step_), X(curr_step_), B(prev_step_), tfs_.target_T_dvl,
      dvl_tight_preintegrator_->delta(), dvl_tight_preintegrator_->preintMeasDerivativeWrtBias(),
      prev_imu_bias_.gyroscope(), preint_noise);
}

void FactorGraphCore::addOriginDeltaPriorFactor(gtsam::NonlinearFactorGraph& graph,
                                                gtsam::Values& values, size_t agent_queue_idx,
                                                const utils::AgentStatusData& msg) {
  const auto& priors = params_.priors;

  gtsam::Rot3 map_R_delta =
      gtsam::Rot3::Ypr(priors.origin_delta_orientation[2], priors.origin_delta_orientation[1],
                       priors.origin_delta_orientation[0]);
  gtsam::Point3 map_p_delta(priors.origin_delta_position[0], priors.origin_delta_position[1],
                            priors.origin_delta_position[2]);
  gtsam::Pose3 delta_prior(map_R_delta, map_p_delta);

  gtsam::Matrix3 map_orientation_cov = sigmasSquaredDiag(priors.origin_delta_orientation_sigmas);
  gtsam::Matrix3 map_position_cov = sigmasSquaredDiag(priors.origin_delta_position_sigmas);

  const gtsam::Matrix3 delta_R_map = map_R_delta.inverse().matrix();

  // Conjugate map-frame pose covariance into the delta-frame tangent space
  gtsam::Matrix6 delta_cov = gtsam::Matrix6::Zero();
  delta_cov.topLeftCorner<3, 3>() = delta_R_map * map_orientation_cov * delta_R_map.transpose();
  delta_cov.bottomRightCorner<3, 3>() = delta_R_map * map_position_cov * delta_R_map.transpose();

  graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
      O(agent_queue_idx), delta_prior, gtsam::noiseModel::Gaussian::Covariance(delta_cov));

  // Use the first range/bearing pair as an xy delta seed to prevent antipodal trapping
  gtsam::Pose3 delta_seed = delta_prior;
  if (msg.includes_range && msg.includes_usbl) {
    const gtsam::Pose3 map_T_modem_l = prev_pose_ * tfs_.target_T_modem;
    const gtsam::Unit3 modem_l_dir_n =
        BearingFactorArm::losDirection(gtsam::Point2(msg.usbl_azimuth, msg.usbl_elevation));

    const gtsam::Point3 modem_l_p_modem_n(msg.range_dist * modem_l_dir_n.point3());
    const gtsam::Point3 map_p_modem_n = map_T_modem_l.transformFrom(modem_l_p_modem_n);

    // Account for the neighbor's modem lever arm
    const gtsam::Rot3 map_R_n = map_R_delta * msg.pose.rotation();
    const gtsam::Point3 map_p_n = map_p_modem_n - map_R_n * neighbor_base_T_modem_.translation();

    const gtsam::Point3 map_p_delta_los = map_p_n - map_R_delta * msg.pose.translation();
    delta_seed = gtsam::Pose3(
        map_R_delta, gtsam::Point3(map_p_delta_los.x(), map_p_delta_los.y(), map_p_delta.z()));
  }

  values.insert(O(agent_queue_idx), delta_seed);
  prev_origin_deltas_[agent_queue_idx] = delta_seed;
}

void FactorGraphCore::addNeighborPriorFactor(gtsam::NonlinearFactorGraph& graph,
                                             const NeighborState& neighbor,
                                             size_t agent_queue_idx) {
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
      N(neighbor.curr_step), neighbor.curr_pose,
      gtsam::noiseModel::Gaussian::Covariance(neighbor.curr_cov));

  gtsam::Vector6 prior_pose_sigmas = neighbor.curr_cov.diagonal().cwiseSqrt();

  std::ostringstream oss;
  oss << "Initial neighbor state (queue " << agent_queue_idx << ", t=" << std::fixed
      << std::setprecision(4) << neighbor.curr_time << "):\n";
  oss << std::scientific;
  oss << "  Position [m]        : " << neighbor.curr_pose.translation().transpose() << "\n"
      << "  Orientation [rad]   : " << neighbor.curr_pose.rotation().rpy().transpose() << " (RPY)\n"
      << "Prior sigmas:\n"
      << "  Position [m]        : " << prior_pose_sigmas.tail<3>().transpose() << "\n"
      << "  Orientation [rad]   : " << prior_pose_sigmas.head<3>().transpose() << " (RPY)";
  logger_.log(utils::LogLevel::kInfo, oss.str());
}

void FactorGraphCore::addNeighborBetweenFactor(gtsam::NonlinearFactorGraph& graph,
                                               const NeighborState& neighbor) {
  // Origin state method derived from Walls et al., IEEE ICRA 2015
  gtsam::Matrix66 H_prev;
  gtsam::Matrix66 H_curr;
  gtsam::Pose3 prev_T_curr = neighbor.prev_pose.between(neighbor.curr_pose, H_prev, H_curr);

  // Approximate relative covariance (ignores cross-correlation between the two chain poses)
  gtsam::Matrix66 between_cov = H_prev * neighbor.prev_cov * H_prev.transpose() +
                                H_curr * neighbor.curr_cov * H_curr.transpose();

  graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
      N(neighbor.curr_step - 1), N(neighbor.curr_step), prev_T_curr,
      gtsam::noiseModel::Gaussian::Covariance(between_cov));
}

void FactorGraphCore::addNeighborDepthFactor(gtsam::NonlinearFactorGraph& graph,
                                             const utils::AgentStatusData& msg,
                                             const NeighborState& neighbor,
                                             size_t agent_queue_idx) {
  const gtsam::Pose3 kNoArm;

  const double depth_sigma = params_.multiagent.neighbor.depth.position_z_noise_sigma *
                             std::sqrt(params_.multiagent.neighbor.depth.covariance_scalar);
  gtsam::SharedNoiseModel depth_noise = gtsam::noiseModel::Isotropic::Sigma(1, depth_sigma);

  depth_noise = applyRobustKernel(depth_noise, params_.multiagent.neighbor.depth.robust_kernel,
                                  params_.multiagent.neighbor.depth.robust_k);

  if (params_.multiagent.estimate_origin_delta) {
    graph.emplace_shared<DepthOriginDeltaFactorArm>(O(agent_queue_idx), N(neighbor.curr_step),
                                                    msg.pressure_depth, kNoArm, depth_noise);
  } else {
    graph.emplace_shared<DepthFactorArm>(N(neighbor.curr_step), msg.pressure_depth, kNoArm,
                                         depth_noise);
  }
}

void FactorGraphCore::addNeighborAhrsFactor(gtsam::NonlinearFactorGraph& graph,
                                            const utils::AgentStatusData& msg,
                                            const NeighborState& neighbor, size_t agent_queue_idx) {
  const gtsam::Pose3 kNoArm;

  const gtsam::Matrix3 map_ahrs_cov =
      sigmasSquaredDiag<3>(params_.multiagent.neighbor.ahrs.orientation_noise_sigmas) *
      params_.multiagent.neighbor.ahrs.covariance_scalar;

  // Conjugate map-frame orientation covariance into the sensor-frame tangent space
  gtsam::SharedNoiseModel ahrs_noise = gtsam::noiseModel::Gaussian::Covariance(
      AhrsFactorArm::sensorTangentCovariance(map_ahrs_cov, msg.imu_orientation));

  ahrs_noise = applyRobustKernel(ahrs_noise, params_.multiagent.neighbor.ahrs.robust_kernel,
                                 params_.multiagent.neighbor.ahrs.robust_k);

  if (params_.multiagent.estimate_origin_delta) {
    graph.emplace_shared<AhrsOriginDeltaFactorArm>(
        O(agent_queue_idx), N(neighbor.curr_step), msg.imu_orientation, kNoArm,
        params_.multiagent.neighbor.ahrs.mag_declination_radians, ahrs_noise);
  } else {
    graph.emplace_shared<AhrsFactorArm>(N(neighbor.curr_step), msg.imu_orientation, kNoArm,
                                        params_.multiagent.neighbor.ahrs.mag_declination_radians,
                                        ahrs_noise);
  }
}

void FactorGraphCore::addInterAgentRangeFactor(gtsam::NonlinearFactorGraph& graph,
                                               const utils::AgentStatusData& msg,
                                               const NeighborState& neighbor, gtsam::Key pose_key,
                                               size_t agent_queue_idx) {
  if (!msg.includes_range) {
    return;
  }

  const double range_sigma =
      params_.multiagent.range_noise_sigma * std::sqrt(params_.multiagent.covariance_scalar);
  gtsam::SharedNoiseModel range_noise = gtsam::noiseModel::Isotropic::Sigma(1, range_sigma);

  range_noise =
      applyRobustKernel(range_noise, params_.multiagent.robust_kernel, params_.multiagent.robust_k);

  if (params_.multiagent.estimate_origin_delta) {
    graph.emplace_shared<RangeOriginDeltaFactorArm>(
        pose_key, O(agent_queue_idx), N(neighbor.curr_step), msg.range_dist, tfs_.target_T_modem,
        neighbor_base_T_modem_, range_noise);
  } else {
    graph.emplace_shared<RangeFactorArm>(pose_key, N(neighbor.curr_step), msg.range_dist,
                                         tfs_.target_T_modem, neighbor_base_T_modem_, range_noise);
  }
}

void FactorGraphCore::addInterAgentBearingFactor(gtsam::NonlinearFactorGraph& graph,
                                                 const utils::AgentStatusData& msg,
                                                 const NeighborState& neighbor, gtsam::Key pose_key,
                                                 size_t agent_queue_idx) {
  if (!msg.includes_usbl) {
    return;
  }

  const gtsam::Point2 measured_azi_el(msg.usbl_azimuth, msg.usbl_elevation);

  const gtsam::Matrix22 azi_el_cov = sigmasSquaredDiag<2>(params_.multiagent.bearing_noise_sigmas) *
                                     params_.multiagent.covariance_scalar;

  // Conjugate azimuth/elevation covariance into the residual's Unit3 tangent space
  gtsam::SharedNoiseModel bearing_noise = gtsam::noiseModel::Gaussian::Covariance(
      BearingFactorArm::unit3TangentCovariance(azi_el_cov, measured_azi_el));

  bearing_noise = applyRobustKernel(bearing_noise, params_.multiagent.robust_kernel,
                                    params_.multiagent.robust_k);

  if (params_.multiagent.estimate_origin_delta) {
    graph.emplace_shared<BearingOriginDeltaFactorArm>(
        pose_key, O(agent_queue_idx), N(neighbor.curr_step), measured_azi_el, tfs_.target_T_modem,
        neighbor_base_T_modem_, bearing_noise);
  } else {
    graph.emplace_shared<BearingFactorArm>(pose_key, N(neighbor.curr_step), measured_azi_el,
                                           tfs_.target_T_modem, neighbor_base_T_modem_,
                                           bearing_noise);
  }
}

void FactorGraphCore::addMultiAgentFactors(
    gtsam::NonlinearFactorGraph& graph, gtsam::Values& values,
    gtsam::IncrementalFixedLagSmoother::KeyTimestampMap& timestamps,
    const utils::QueueBundle& queues, double target_time) {
  // Conjugate a neighbor's broadcast pose covariance into its base-frame tangent space
  auto base_tangent_cov = [](const utils::AgentStatusData& msg) {
    gtsam::Matrix66 map_R_base = gtsam::Matrix66::Zero();
    map_R_base.block<3, 3>(0, 0) = msg.pose.rotation().matrix();
    map_R_base.block<3, 3>(3, 3) = msg.pose.rotation().matrix();

    return gtsam::Matrix66(map_R_base.transpose() * msg.pose_covariance * map_R_base);
  };

  auto nearest_pose_key = [this, target_time](double stamp) {
    const auto stamp_ns = static_cast<int64_t>(stamp * kSecondsToNanoseconds);
    const auto after = time_to_key_.lower_bound(stamp_ns);

    gtsam::Key key = X(curr_step_);
    int64_t best = std::llabs(static_cast<int64_t>(target_time * kSecondsToNanoseconds) - stamp_ns);

    if (after != time_to_key_.end() && after->first - stamp_ns < best) {
      best = after->first - stamp_ns;
      key = after->second;
    }
    if (after != time_to_key_.begin() && stamp_ns - std::prev(after)->first < best) {
      key = std::prev(after)->second;
    }
    return key;
  };

  for (size_t agent_queue_idx = 0; agent_queue_idx < queues.multiagent.size(); ++agent_queue_idx) {
    const auto& queue = queues.multiagent[agent_queue_idx];
    if (queue.empty()) {
      continue;
    }

    const auto& msg = queue.back();

    if (!msg->pose_covariance.allFinite() ||
        (msg->pose_covariance.diagonal().array() <= 0.0).any()) {
      logger_.log(
          utils::LogLevel::kWarn,
          "Neighbor status covariance (queue " + std::to_string(agent_queue_idx) +
              ") is unusable (non-finite or non-positive diagonal); dropping the keyframe.");
      continue;
    }

    auto [it, inserted] = neighbors_.try_emplace(agent_queue_idx, NeighborState(agent_queue_idx));
    auto& neighbor = it->second;

    if (inserted && params_.multiagent.estimate_origin_delta) {
      if (!msg->includes_range || !msg->includes_usbl) {
        logger_.log(utils::LogLevel::kWarn,
                    "Neighbor status (queue " + std::to_string(agent_queue_idx) +
                        ") has no range/bearing pair to seed the origin delta; dropping "
                        "the keyframe.");
        neighbors_.erase(it);
        continue;
      }
      addOriginDeltaPriorFactor(graph, values, agent_queue_idx, *msg);
    }

    const gtsam::Matrix66 base_cov = base_tangent_cov(*msg);

    // Subtract the acoustic flight time to determine when the neighbor replied
    const double sent_time = msg->includes_range
                                 ? msg->timestamp - msg->range_dist / params_.multiagent.sound_speed
                                 : msg->timestamp;

    if (inserted) {
      neighbor.initialize(msg->pose, base_cov, sent_time);
      addNeighborPriorFactor(graph, neighbor, agent_queue_idx);
    } else {
      // Handle acomms dropouts longer than the smoother lag
      const bool expired =
          inc_smoother_ && (target_time - neighbor.curr_time) > params_.smoother_lag_sec;

      neighbor.advance(msg->pose, base_cov, sent_time);

      if (expired) {
        addNeighborPriorFactor(graph, neighbor, agent_queue_idx);
      } else {
        addNeighborBetweenFactor(graph, neighbor);
      }
    }

    values.insert(N(neighbor.curr_step), neighbor.curr_pose);
    timestamps[N(neighbor.curr_step)] = sent_time;

    if (params_.multiagent.neighbor.depth.enable_depth) {
      addNeighborDepthFactor(graph, *msg, neighbor, agent_queue_idx);
    }
    if (params_.multiagent.neighbor.ahrs.enable_ahrs) {
      addNeighborAhrsFactor(graph, *msg, neighbor, agent_queue_idx);
    }

    const gtsam::Key pose_key = nearest_pose_key(sent_time);
    if (params_.multiagent.enable_range) {
      addInterAgentRangeFactor(graph, *msg, neighbor, pose_key, agent_queue_idx);
    }
    if (params_.multiagent.enable_bearing) {
      addInterAgentBearingFactor(graph, *msg, neighbor, pose_key, agent_queue_idx);
    }
  }
}

std::optional<utils::QueueBundle> FactorGraphCore::update(double target_time,
                                                          utils::QueueBundle& queues,
                                                          const utils::TfBundle& tfs) {
  if (target_time <= prev_time_ + kMinIntegrationDt) {
    return std::nullopt;
  }

  // Sort sensor messages by timestamp
  auto by_time = [](const auto& a, const auto& b) { return a->timestamp < b->timestamp; };
  std::sort(queues.imu.begin(), queues.imu.end(), by_time);
  std::sort(queues.gps.begin(), queues.gps.end(), by_time);
  std::sort(queues.depth.begin(), queues.depth.end(), by_time);
  std::sort(queues.mag.begin(), queues.mag.end(), by_time);
  std::sort(queues.ahrs.begin(), queues.ahrs.end(), by_time);
  std::sort(queues.dvl.begin(), queues.dvl.end(), by_time);
  std::sort(queues.wrench.begin(), queues.wrench.end(), by_time);
  for (auto& agent : queues.multiagent) {
    std::sort(agent.begin(), agent.end(), by_time);
  }

  if (queues.imu.empty() || queues.imu.front()->timestamp > target_time) {
    logger_.log(utils::LogLevel::kWarn,
                "Keyframe rejected: no IMU measurements at or before the keyframe time.");
    return std::nullopt;
  }

  // --- Build Factor Graph ---
  gtsam::NonlinearFactorGraph new_graph;
  gtsam::Values new_values;
  gtsam::IncrementalFixedLagSmoother::KeyTimestampMap new_timestamps;

  utils::QueueBundle leftover;

  std::scoped_lock state_lock(state_mutex_);

  // Update lazily-resolved transforms
  tfs_ = tfs;

  const gtsam::Vector3 held_imu_accel = last_imu_accel_;
  const gtsam::Vector3 held_imu_gyro = last_imu_gyro_;

  // Re-queue messages newer than the keyframe for the next update
  auto split_after_target = [target_time](auto& msgs, auto& split) {
    while (!msgs.empty() && msgs.back()->timestamp > target_time) {
      split.push_front(msgs.back());
      msgs.pop_back();
    }
  };
  split_after_target(queues.imu, leftover.imu);
  split_after_target(queues.gps, leftover.gps);
  split_after_target(queues.depth, leftover.depth);
  split_after_target(queues.mag, leftover.mag);
  split_after_target(queues.ahrs, leftover.ahrs);
  split_after_target(queues.dvl, leftover.dvl);
  split_after_target(queues.wrench, leftover.wrench);
  leftover.multiagent.resize(queues.multiagent.size());
  for (size_t agent_queue_idx = 0; agent_queue_idx < queues.multiagent.size(); ++agent_queue_idx) {
    split_after_target(queues.multiagent[agent_queue_idx], leftover.multiagent[agent_queue_idx]);
  }

  if (params_.comparison.enable_loose_dvl_preintegration && !queues.ahrs.empty()) {
    leftover.ahrs.push_front(queues.ahrs.back());
  }

  addImuPreintFactor(new_graph, queues.imu, target_time);
  if (params_.gps.enable_gps) {
    addGpsFactor(new_graph, queues.gps);
  }
  if (params_.depth.enable_depth) {
    addDepthFactor(new_graph, queues.depth);
  }
  if (params_.mag.enable_mag) {
    addMagFactor(new_graph, queues.mag);
  }
  if (params_.ahrs.enable_ahrs) {
    addAhrsFactor(new_graph, queues.ahrs);
  }

  // Handle DVL dropouts
  auto add_dropout_factors = [&](gtsam::NonlinearFactorGraph& g) {
    bool use_wrench = params_.wrench.enable_wrench || params_.wrench.enable_wrench_dropout_only;
    bool use_const_vel =
        params_.const_vel.enable_const_vel || params_.const_vel.enable_const_vel_dropout_only;

    if (use_wrench) {
      addWrenchDynamicsFactor(g, queues.wrench, target_time);
    } else if (use_const_vel) {
      addConstVelFactor(g, target_time);
    }
  };

  if (queues.dvl.empty() || !params_.dvl.enable_dvl) {
    add_dropout_factors(new_graph);
  } else {
    if (params_.comparison.enable_loose_dvl_preintegration) {
      if (queues.ahrs.empty()) {
        add_dropout_factors(new_graph);
        last_dvl_vel_ = queues.dvl.back()->linear_velocity;
      } else {
        addDvlLoosePreintFactor(new_graph, queues.dvl, queues.ahrs, target_time);
      }
    } else if (params_.comparison.enable_tight_dvl_preintegration) {
      addDvlTightPreintFactor(new_graph, queues.dvl, queues.imu, target_time, held_imu_accel,
                              held_imu_gyro);
    } else {
      addDvlFactor(new_graph, queues.dvl, last_imu_gyro_);

      if (params_.wrench.enable_wrench) {
        addWrenchDynamicsFactor(new_graph, queues.wrench, target_time);
      } else if (params_.const_vel.enable_const_vel) {
        addConstVelFactor(new_graph, target_time);
      }
    }
  }

  if (params_.multiagent.enable_multiagent) {
    addMultiAgentFactors(new_graph, new_values, new_timestamps, queues, target_time);
  }

  // --- Add State Predictions ---
  auto pred = imu_preintegrator_->predict(gtsam::NavState(prev_pose_, prev_vel_), prev_imu_bias_);
  new_values.insert(X(curr_step_), pred.pose());
  new_values.insert(V(curr_step_), pred.velocity());
  new_values.insert(B(curr_step_), prev_imu_bias_);
  new_timestamps[X(curr_step_)] = target_time;
  new_timestamps[V(curr_step_)] = target_time;
  new_timestamps[B(curr_step_)] = target_time;

  if (params_.mag.estimate_hard_iron_bias) {
    new_timestamps[M(0)] = target_time;
  }

  for (const auto& [agent_queue_idx, delta] : prev_origin_deltas_) {
    new_timestamps[O(agent_queue_idx)] = target_time;
  }

  if (!inc_smoother_ && !isam_) {
    prev_pose_ = pred.pose();
    prev_vel_ = pred.velocity();
  }

  // --- Reset Preintegrators ---
  imu_preintegrator_->resetIntegrationAndSetBias(prev_imu_bias_);

  if (params_.publish_smoothed_path || params_.multiagent.enable_multiagent) {
    time_to_key_[static_cast<int64_t>(target_time * kSecondsToNanoseconds)] = X(curr_step_);
    if (inc_smoother_) {
      time_to_key_.erase(time_to_key_.begin(),
                         time_to_key_.lower_bound(static_cast<int64_t>(
                             (target_time - params_.smoother_lag_sec) * kSecondsToNanoseconds)));
    }
  }

  prev_time_ = target_time;
  prev_step_ = curr_step_;
  curr_step_++;

  // --- Add Graph to Buffer ---
  buffer_graph_ += new_graph;
  buffer_values_.insert(new_values);
  for (const auto& [key, stamp] : new_timestamps) {
    buffer_timestamps_.insert_or_assign(key, stamp);
  }
  buffer_target_time_ = target_time;
  buffer_prev_step_ = prev_step_;
  buffer_keyframes_++;
  has_buffer_ = true;

  return leftover;
}

std::optional<OptimizeResult> FactorGraphCore::optimize() {
  // --- Load Graph from Buffer ---
  gtsam::NonlinearFactorGraph batch_graph;
  gtsam::Values batch_values;
  gtsam::IncrementalFixedLagSmoother::KeyTimestampMap batch_timestamps;
  double batch_target_time{0.0};
  size_t batch_prev_step = 0;
  size_t batch_keyframes = 0;

  {
    std::scoped_lock state_lock(state_mutex_);
    if (!has_buffer_) {
      return std::nullopt;
    }

    batch_graph = std::move(buffer_graph_);
    batch_values = std::move(buffer_values_);
    batch_timestamps = std::move(buffer_timestamps_);
    batch_target_time = buffer_target_time_;
    batch_prev_step = buffer_prev_step_;
    batch_keyframes = buffer_keyframes_;

    buffer_graph_ = gtsam::NonlinearFactorGraph();
    buffer_values_ = gtsam::Values();
    buffer_timestamps_.clear();
    buffer_keyframes_ = 0;
    has_buffer_ = false;
  }

  OptimizeResult result;
  result.timestamp = batch_target_time;

  // --- Detect Processing Overflow ---
  result.new_keyframes = batch_keyframes;
  if (result.new_keyframes > 1) {
    result.processing_overflow = true;
  }

  // --- Smoother Optimization ---
  auto total_start = std::chrono::steady_clock::now();
  result.new_factors = batch_graph.size();

  if (inc_smoother_) {
    auto smoother_start = std::chrono::steady_clock::now();
    inc_smoother_->update(batch_graph, batch_values, batch_timestamps);
    auto smoother_end = std::chrono::steady_clock::now();
    result.smoother_duration = std::chrono::duration<double>(smoother_end - smoother_start).count();

    {
      std::scoped_lock state_lock(state_mutex_);
      prev_pose_ = inc_smoother_->calculateEstimate<gtsam::Pose3>(X(batch_prev_step));
      prev_vel_ = inc_smoother_->calculateEstimate<gtsam::Vector3>(V(batch_prev_step));
      prev_imu_bias_ =
          inc_smoother_->calculateEstimate<gtsam::imuBias::ConstantBias>(B(batch_prev_step));
      if (params_.mag.estimate_hard_iron_bias) {
        prev_mag_bias_ = inc_smoother_->calculateEstimate<gtsam::Point3>(M(0));
      }
      for (auto& [agent_queue_idx, delta] : prev_origin_deltas_) {
        delta = inc_smoother_->calculateEstimate<gtsam::Pose3>(O(agent_queue_idx));
      }
    }

    if (params_.publish_diagnostics || params_.publish_graph_metrics) {
      result.total_factors = inc_smoother_->getFactors().nrFactors();
      result.total_variables = inc_smoother_->getLinearizationPoint().size();
    }

  } else if (isam_) {
    auto smoother_start = std::chrono::steady_clock::now();
    isam_->update(batch_graph, batch_values);
    auto smoother_end = std::chrono::steady_clock::now();
    result.smoother_duration = std::chrono::duration<double>(smoother_end - smoother_start).count();

    {
      std::scoped_lock state_lock(state_mutex_);
      prev_pose_ = isam_->calculateEstimate<gtsam::Pose3>(X(batch_prev_step));
      prev_vel_ = isam_->calculateEstimate<gtsam::Vector3>(V(batch_prev_step));
      prev_imu_bias_ = isam_->calculateEstimate<gtsam::imuBias::ConstantBias>(B(batch_prev_step));
      if (params_.mag.estimate_hard_iron_bias) {
        prev_mag_bias_ = isam_->calculateEstimate<gtsam::Point3>(M(0));
      }
      for (auto& [agent_queue_idx, delta] : prev_origin_deltas_) {
        delta = isam_->calculateEstimate<gtsam::Pose3>(O(agent_queue_idx));
      }
    }

    if (params_.publish_diagnostics || params_.publish_graph_metrics) {
      result.total_factors = isam_->getFactorsUnsafe().nrFactors();
      result.total_variables = isam_->getLinearizationPoint().size();
    }
  } else {
    lm_graph_.push_back(batch_graph.begin(), batch_graph.end());
    lm_values_.insert(batch_values);

    auto smoother_start = std::chrono::steady_clock::now();
    gtsam::LevenbergMarquardtParams lm_params;
    gtsam::LevenbergMarquardtOptimizer optimizer(lm_graph_, lm_values_, lm_params);
    lm_values_ = optimizer.optimize();
    auto smoother_end = std::chrono::steady_clock::now();
    result.smoother_duration = std::chrono::duration<double>(smoother_end - smoother_start).count();

    {
      std::scoped_lock state_lock(state_mutex_);
      prev_pose_ = lm_values_.at<gtsam::Pose3>(X(batch_prev_step));
      prev_vel_ = lm_values_.at<gtsam::Vector3>(V(batch_prev_step));
      prev_imu_bias_ = lm_values_.at<gtsam::imuBias::ConstantBias>(B(batch_prev_step));
      if (params_.mag.estimate_hard_iron_bias) {
        prev_mag_bias_ = lm_values_.at<gtsam::Point3>(M(0));
      }
      for (auto& [agent_queue_idx, delta] : prev_origin_deltas_) {
        delta = lm_values_.at<gtsam::Pose3>(O(agent_queue_idx));
      }
    }

    if (params_.publish_diagnostics || params_.publish_graph_metrics) {
      result.total_factors = lm_graph_.nrFactors();
      result.total_variables = lm_values_.size();
    }
  }

  {
    std::scoped_lock state_lock(state_mutex_);

    result.pose = prev_pose_;
    result.velocity = prev_vel_;
    result.imu_bias = prev_imu_bias_;
    result.mag_bias = prev_mag_bias_;

    result.neighbor_estimates.reserve(neighbors_.size());
    for (const auto& [agent_queue_idx, neighbor] : neighbors_) {
      NeighborEstimate estimate;
      estimate.agent_queue_idx = agent_queue_idx;
      estimate.timestamp = neighbor.curr_time;
      estimate.pose_key = N(neighbor.curr_step);

      if (inc_smoother_ && inc_smoother_->getLinearizationPoint().exists(estimate.pose_key)) {
        estimate.pose = inc_smoother_->calculateEstimate<gtsam::Pose3>(estimate.pose_key);
      } else if (isam_ && isam_->getLinearizationPoint().exists(estimate.pose_key)) {
        estimate.pose = isam_->calculateEstimate<gtsam::Pose3>(estimate.pose_key);
      } else if (lm_values_.exists(estimate.pose_key)) {
        estimate.pose = lm_values_.at<gtsam::Pose3>(estimate.pose_key);
      } else {
        // Key was marginalized out of the smoother lag
        continue;
      }

      // Transform the neighbor's pose into the map frame with the origin delta
      if (params_.multiagent.estimate_origin_delta) {
        estimate.pose = prev_origin_deltas_.at(agent_queue_idx) * estimate.pose;
      }

      result.neighbor_estimates.push_back(std::move(estimate));
    }
  }

  // --- Calculate Covariances ---
  auto cov_start = std::chrono::steady_clock::now();

  auto marginal_cov = [&](bool enabled, gtsam::Key key, int dim) -> gtsam::Matrix {
    if (enabled) {
      if (inc_smoother_) {
        return inc_smoother_->marginalCovariance(key);
      }
      if (isam_) {
        return isam_->marginalCovariance(key);
      }
    }
    static constexpr double kUnknownCovariance = -1.0;
    return gtsam::Matrix::Identity(dim, dim) * kUnknownCovariance;
  };

  result.pose_cov = marginal_cov(params_.publish_pose_cov, X(batch_prev_step), 6);
  result.velocity_cov =
      marginal_cov(params_.publish_velocity && params_.publish_velocity_cov, V(batch_prev_step), 3);
  result.imu_bias_cov =
      marginal_cov(params_.publish_imu_bias && params_.publish_imu_bias_cov, B(batch_prev_step), 6);
  result.mag_bias_cov = marginal_cov(params_.mag.estimate_hard_iron_bias &&
                                         params_.publish_mag_bias && params_.publish_mag_bias_cov,
                                     M(0), 3);

  // Neighbor poses are published as (origin delta * neighbor), so use the joint over both keys
  const gtsam::Values* cov_values = nullptr;
  std::optional<gtsam::Marginals> cov_marginals;
  if (params_.publish_neighbor_pose_cov && params_.multiagent.estimate_origin_delta &&
      !result.neighbor_estimates.empty()) {
    if (inc_smoother_) {
      cov_values = &inc_smoother_->getLinearizationPoint();
      cov_marginals.emplace(inc_smoother_->getFactors(), *cov_values);
    } else if (isam_) {
      cov_values = &isam_->getLinearizationPoint();
      cov_marginals.emplace(isam_->getFactorsUnsafe(), *cov_values);
    }
  }

  auto neighbor_cov = [&](gtsam::Key pose_key, size_t agent_queue_idx) -> gtsam::Matrix {
    const gtsam::Key delta_key = O(agent_queue_idx);

    if (!cov_marginals || !cov_values->exists(delta_key) || !cov_values->exists(pose_key)) {
      return marginal_cov(params_.publish_neighbor_pose_cov, pose_key, 6);
    }

    gtsam::Matrix66 H_delta;
    gtsam::Matrix66 H_neighbor;
    cov_values->at<gtsam::Pose3>(delta_key).compose(cov_values->at<gtsam::Pose3>(pose_key), H_delta,
                                                    H_neighbor);

    const gtsam::JointMarginal joint =
        cov_marginals->jointMarginalCovariance(gtsam::KeyVector{delta_key, pose_key});

    // Propagate the joint through the composition, keeping the cross-correlation
    const gtsam::Matrix66 cross = H_delta * joint.at(delta_key, pose_key) * H_neighbor.transpose();
    return H_delta * joint.at(delta_key, delta_key) * H_delta.transpose() +
           H_neighbor * joint.at(pose_key, pose_key) * H_neighbor.transpose() + cross +
           cross.transpose();
  };

  for (auto& neighbor : result.neighbor_estimates) {
    neighbor.pose_cov = neighbor_cov(neighbor.pose_key, neighbor.agent_queue_idx);
  }

  auto cov_end = std::chrono::steady_clock::now();
  result.cov_duration = std::chrono::duration<double>(cov_end - cov_start).count();

  // --- Export Smoothed Path ---
  if (params_.publish_smoothed_path) {
    if (inc_smoother_) {
      result.all_estimates = inc_smoother_->calculateEstimate();
    } else if (isam_) {
      result.all_estimates = isam_->calculateEstimate();
    } else {
      result.all_estimates = lm_values_;
    }
  }

  auto total_end = std::chrono::steady_clock::now();
  result.total_duration = std::chrono::duration<double>(total_end - total_start).count();

  return result;
}

std::map<int64_t, gtsam::Key> FactorGraphCore::snapshotTimeKeys() const {
  std::scoped_lock lock(state_mutex_);
  return time_to_key_;
}

}  // namespace coug_fg
