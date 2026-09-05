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
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace coug_fg::factors {

class DvlFactorArm
    : public gtsam::NoiseModelFactor3<gtsam::Pose3, gtsam::Vector3, gtsam::imuBias::ConstantBias> {
  gtsam::Vector3 measured_velocity_;
  gtsam::Vector3 measured_gyro_;
  gtsam::Rot3 target_R_sensor_;
  gtsam::Point3 target_p_sensor_;
  gtsam::Rot3 target_R_imu_;

 public:
  static auto gyroJacobian(const gtsam::Rot3& target_R_sensor, const gtsam::Point3& target_p_sensor,
                           const gtsam::Rot3& target_R_imu) -> gtsam::Matrix3 {
    return target_R_sensor.transpose() * gtsam::skewSymmetric(target_p_sensor) *
           target_R_imu.matrix();
  }

  static auto gyroLeverArmCovariance(const gtsam::Matrix3& gyro_sample_cov,
                                     const gtsam::Pose3& target_T_sensor,
                                     const gtsam::Pose3& target_T_imu) -> gtsam::Matrix3 {
    const gtsam::Matrix3 J_vel_gyro = gyroJacobian(
        target_T_sensor.rotation(), target_T_sensor.translation(), target_T_imu.rotation());

    return J_vel_gyro * gyro_sample_cov * J_vel_gyro.transpose();
  }

  DvlFactorArm(gtsam::Key pose_key, gtsam::Key vel_key, gtsam::Key bias_key,
               const gtsam::Pose3& target_T_sensor, const gtsam::Pose3& target_T_imu,
               const gtsam::Vector3& measured_velocity, const gtsam::Vector3& measured_gyro,
               const gtsam::SharedNoiseModel& noise_model)
      : NoiseModelFactor3<gtsam::Pose3, gtsam::Vector3, gtsam::imuBias::ConstantBias>(
            noise_model, pose_key, vel_key, bias_key),
        measured_velocity_(measured_velocity),
        measured_gyro_(measured_gyro),
        target_R_sensor_(target_T_sensor.rotation()),
        target_p_sensor_(target_T_sensor.translation()),
        target_R_imu_(target_T_imu.rotation()) {}

  auto evaluateError(const gtsam::Pose3& pose, const gtsam::Vector3& map_v_target,
                     const gtsam::imuBias::ConstantBias& bias,
                     gtsam::OptionalMatrixType H_pose = nullptr,
                     gtsam::OptionalMatrixType H_vel = nullptr,
                     gtsam::OptionalMatrixType H_bias = nullptr) const -> gtsam::Vector override {
    gtsam::Matrix33 H_unrotate_R = gtsam::Matrix33::Zero();
    gtsam::Matrix33 H_unrotate_v = gtsam::Matrix33::Zero();

    const gtsam::Vector3 target_vel = pose.rotation().unrotate(
        map_v_target, H_pose ? &H_unrotate_R : nullptr, H_vel ? &H_unrotate_v : nullptr);

    const gtsam::Vector3 target_omega = target_R_imu_.rotate(measured_gyro_ - bias.gyroscope());
    const gtsam::Vector3 target_v_lever_arm = target_omega.cross(target_p_sensor_);

    const gtsam::Vector3 predicted_velocity =
        target_R_sensor_.unrotate(target_vel + target_v_lever_arm);

    // 3D velocity residual
    const gtsam::Vector3 error = predicted_velocity - measured_velocity_;

    if (H_pose) {
      // Jacobian with respect to pose (3x6)
      H_pose->setZero(3, 6);
      H_pose->block<3, 3>(0, 0) = target_R_sensor_.transpose() * H_unrotate_R;
    }

    if (H_vel) {
      // Jacobian with respect to velocity (3x3)
      *H_vel = target_R_sensor_.transpose() * H_unrotate_v;
    }

    if (H_bias) {
      // Jacobian with respect to bias (3x6)
      H_bias->setZero(3, 6);
      H_bias->block<3, 3>(0, 3) = gyroJacobian(target_R_sensor_, target_p_sensor_, target_R_imu_);
    }

    return error;
  }
};

}  // namespace coug_fg::factors
