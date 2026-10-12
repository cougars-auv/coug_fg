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

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include "coug_fg/factors/dvl_factor.hpp"

namespace coug_fg::factors {

class DvlBeamFactorArm
    : public gtsam::NoiseModelFactor3<gtsam::Pose3, gtsam::Vector3, gtsam::imuBias::ConstantBias> {
  double measured_velocity_;
  gtsam::Vector3 measured_gyro_;
  gtsam::Rot3 target_R_sensor_;
  gtsam::Point3 target_p_sensor_;
  gtsam::Rot3 target_R_imu_;
  gtsam::Vector3 target_beam_axis_;

 public:
  static auto gyroLeverArmVariance(const gtsam::Matrix3& gyro_sample_cov,
                                   const gtsam::Pose3& target_T_sensor,
                                   const gtsam::Pose3& target_T_imu) -> double {
    return DvlFactorArm::gyroLeverArmCovariance(gyro_sample_cov, target_T_sensor, target_T_imu)(0,
                                                                                                0);
  }

  DvlBeamFactorArm(gtsam::Key pose_key, gtsam::Key vel_key, gtsam::Key bias_key,
                   const gtsam::Pose3& target_T_sensor, const gtsam::Pose3& target_T_imu,
                   double measured_velocity, const gtsam::Vector3& measured_gyro,
                   const gtsam::SharedNoiseModel& noise_model)
      : NoiseModelFactor3<gtsam::Pose3, gtsam::Vector3, gtsam::imuBias::ConstantBias>(
            noise_model, pose_key, vel_key, bias_key),
        measured_velocity_(measured_velocity),
        measured_gyro_(measured_gyro),
        target_R_sensor_(target_T_sensor.rotation()),
        target_p_sensor_(target_T_sensor.translation()),
        target_R_imu_(target_T_imu.rotation()),
        target_beam_axis_(target_R_sensor_.matrix().col(0)) {}

  auto evaluateError(const gtsam::Pose3& pose, const gtsam::Vector3& map_v_target,
                     const gtsam::imuBias::ConstantBias& bias,
                     gtsam::OptionalMatrixType H_pose = nullptr,
                     gtsam::OptionalMatrixType H_vel = nullptr,
                     gtsam::OptionalMatrixType H_bias = nullptr) const -> gtsam::Vector override {
    gtsam::Matrix33 H_unrotate_R = gtsam::Matrix33::Zero();
    gtsam::Matrix33 H_unrotate_v = gtsam::Matrix33::Zero();

    const gtsam::Vector3 target_vel =
        pose.rotation().unrotate(map_v_target, (H_pose != nullptr) ? &H_unrotate_R : nullptr,
                                 (H_vel != nullptr) ? &H_unrotate_v : nullptr);

    const gtsam::Vector3 target_omega = target_R_imu_.rotate(measured_gyro_ - bias.gyroscope());
    const gtsam::Vector3 target_v_lever_arm = target_omega.cross(target_p_sensor_);

    const double predicted_velocity = target_beam_axis_.dot(target_vel + target_v_lever_arm);

    // 1D along-beam velocity residual
    const gtsam::Vector1 error(predicted_velocity - measured_velocity_);

    if (H_pose != nullptr) {
      // Jacobian with respect to pose (1x6)
      H_pose->setZero(1, 6);
      H_pose->block<1, 3>(0, 0) = target_beam_axis_.transpose() * H_unrotate_R;
    }

    if (H_vel != nullptr) {
      // Jacobian with respect to velocity (1x3)
      *H_vel = target_beam_axis_.transpose() * H_unrotate_v;
    }

    if (H_bias != nullptr) {
      // Jacobian with respect to bias (1x6)
      H_bias->setZero(1, 6);
      H_bias->block<1, 3>(0, 3) =
          DvlFactorArm::gyroJacobian(target_R_sensor_, target_p_sensor_, target_R_imu_).row(0);
    }

    return error;
  }
};

}  // namespace coug_fg::factors
