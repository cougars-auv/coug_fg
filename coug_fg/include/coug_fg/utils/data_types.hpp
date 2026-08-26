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

#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>

#include <Eigen/Dense>
#include <deque>
#include <memory>
#include <vector>

namespace coug_fg::utils {

struct TfBundle {
  gtsam::Pose3 target_T_base;
  gtsam::Pose3 target_T_imu;
  gtsam::Pose3 target_T_gps;
  gtsam::Pose3 target_T_depth;
  gtsam::Pose3 target_T_mag;
  gtsam::Pose3 target_T_ahrs;
  gtsam::Pose3 target_T_dvl;
  gtsam::Pose3 target_T_wrench;
  gtsam::Pose3 target_T_modem;
};

struct ImuData {
  double timestamp{0.0};
  gtsam::Vector3 linear_acceleration;
  gtsam::Vector3 angular_velocity;
  gtsam::Matrix3 linear_acceleration_covariance;
  gtsam::Matrix3 angular_velocity_covariance;
};

struct OdometryData {
  double timestamp{0.0};
  gtsam::Pose3 pose;
  gtsam::Matrix66 pose_covariance;
};

struct MagneticFieldData {
  double timestamp{0.0};
  gtsam::Vector3 magnetic_field;
  gtsam::Matrix3 magnetic_field_covariance;
};

struct AhrsData {
  double timestamp{0.0};
  gtsam::Rot3 orientation;
  gtsam::Matrix3 orientation_covariance;
};

struct TwistData {
  double timestamp{0.0};
  gtsam::Vector3 linear_velocity;
  // Angular-first, to match the GTSAM pose covariance convention
  gtsam::Matrix66 velocity_covariance;
};

struct WrenchData {
  double timestamp{0.0};
  gtsam::Vector3 force;
  gtsam::Vector3 torque;
};

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

}  // namespace coug_fg::utils
