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
 * @file data_types.hpp
 * @brief Shared data types for the factor graph core.
 * @author Nelson Durrant
 * @date May 2026
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/navigation/ImuBias.h>

#include <Eigen/Dense>
#include <deque>
#include <memory>
#include <vector>

namespace coug_fgo::utils {

/**
 * @struct TfBundle
 * @brief Container for static TF sensor transformations.
 */
struct TfBundle {
  gtsam::Pose3 target_T_imu;
  gtsam::Pose3 target_T_gps;
  gtsam::Pose3 target_T_depth;
  gtsam::Pose3 target_T_mag;
  gtsam::Pose3 target_T_ahrs;
  gtsam::Pose3 target_T_dvl;
  gtsam::Pose3 target_T_base;
  gtsam::Pose3 target_T_com;
  gtsam::Pose3 target_T_modem;
};

/**
 * @struct ImuData
 * @brief C++ container for IMU data.
 */
struct ImuData {
  double timestamp{0.0};
  gtsam::Vector3 linear_acceleration;
  gtsam::Vector3 angular_velocity;
  gtsam::Matrix3 linear_acceleration_covariance;
  gtsam::Matrix3 angular_velocity_covariance;
};

/**
 * @struct OdometryData
 * @brief C++ container for Odometry data.
 */
struct OdometryData {
  double timestamp{0.0};
  gtsam::Pose3 pose;
  gtsam::Matrix66 pose_covariance;
};

/**
 * @struct MagneticFieldData
 * @brief C++ container for Magnetometer data.
 */
struct MagneticFieldData {
  double timestamp{0.0};
  gtsam::Vector3 magnetic_field;
  gtsam::Matrix3 magnetic_field_covariance;
};

/**
 * @struct AhrsData
 * @brief C++ container for AHRS data.
 */
struct AhrsData {
  double timestamp{0.0};
  gtsam::Rot3 orientation;
  gtsam::Matrix3 orientation_covariance;
};

/**
 * @struct TwistData
 * @brief C++ container for Twist data.
 */
struct TwistData {
  double timestamp{0.0};
  gtsam::Vector3 linear_velocity;
  gtsam::Matrix66 twist_covariance;
};

/**
 * @struct WrenchData
 * @brief C++ container for Wrench data.
 */
struct WrenchData {
  double timestamp{0.0};
  gtsam::Vector3 force;
  gtsam::Vector3 torque;
};

/**
 * @struct AgentStatusData
 * @brief C++ container for a neighboring agent's broadcast status data.
 */
struct AgentStatusData {
  double timestamp{0.0};
  gtsam::Pose3 pose;
  gtsam::Matrix66 pose_covariance;
  double pressure_depth{0.0};
  gtsam::Rot3 imu_orientation;
  bool includes_range{false};
  double range_dist{0.0};
  bool includes_usbl{false};
  double usbl_azimuth{0.0};
  double usbl_elevation{0.0};
  bool includes_position{false};
  double position_depth{0.0};
};

/**
 * @struct QueueBundle
 * @brief Bundle of drained per-sensor message deques.
 */
struct QueueBundle {
  std::deque<std::shared_ptr<ImuData>> imu;
  std::deque<std::shared_ptr<OdometryData>> gps;
  std::deque<std::shared_ptr<OdometryData>> depth;
  std::deque<std::shared_ptr<MagneticFieldData>> mag;
  std::deque<std::shared_ptr<AhrsData>> ahrs;
  std::deque<std::shared_ptr<TwistData>> dvl;
  std::deque<std::shared_ptr<WrenchData>> wrench;
  std::vector<std::deque<std::shared_ptr<AgentStatusData>>> multiagent;
};

/**
 * @struct InitialState
 * @brief Computed initial state priors and the averaged sensor samples behind them.
 */
struct InitialState {
  gtsam::Pose3 pose;
  gtsam::Vector3 velocity = gtsam::Vector3::Zero();
  gtsam::imuBias::ConstantBias bias;
  double time{0.0};

  std::shared_ptr<ImuData> imu;
  std::shared_ptr<OdometryData> gps;
  std::shared_ptr<OdometryData> depth;
  std::shared_ptr<AhrsData> ahrs;
  std::shared_ptr<MagneticFieldData> mag;
  std::shared_ptr<TwistData> dvl;
};

struct NeighborState {
  explicit NeighborState(uint32_t agent_id, size_t step_spacing = 1000)
      : agent_id(agent_id),
        init_idx(agent_id * step_spacing),
        prev_step(init_idx),
        current_step(init_idx),
        initialized_flag(false) {}

  void Initialize(const gtsam::Pose3& pose, const gtsam::Matrix66& covariance, double time) {
    prev_pose = pose;
    curr_pose = pose;
    prev_covariance = covariance;
    curr_covariance = covariance;
    prev_time = time;
    curr_time = time;
    initialized_flag = true;
  }

  void Advance(const gtsam::Pose3& new_pose, const gtsam::Matrix66& new_covariance,
               double new_time) {
    // Move current estimate to previous
    prev_step = current_step;
    ++current_step;

    prev_pose = curr_pose;
    prev_covariance = curr_covariance;
    prev_time = curr_time;

    // Store new estimate
    curr_pose = new_pose;
    curr_covariance = new_covariance;
    curr_time = new_time;
  }

  uint32_t agent_id;
  const size_t init_idx;

  size_t prev_step;
  size_t current_step;

  double prev_time{0.0};
  double curr_time{0.0};

  gtsam::Pose3 prev_pose;  // Last pose est received from neighbor single agent chain
  gtsam::Pose3 curr_pose;  // Most recent pose received from neighbor single agent chain

  gtsam::Matrix66 prev_covariance = gtsam::Matrix66::Identity();
  gtsam::Matrix66 curr_covariance = gtsam::Matrix66::Identity();

  bool initialized_flag;
};

}  // namespace coug_fgo::utils
