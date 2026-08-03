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
 * @file mag_calib_factor.hpp
 * @brief GTSAM factor for magnetometer measurements that also solves for the hard-iron bias.
 * @author Nelson Durrant
 * @date August 2026
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace coug_fgo::factors {

/**
 * @class MagCalibFactorArm
 * @brief GTSAM factor for magnetometer measurements that also solves for the hard-iron bias.
 */
class MagCalibFactorArm : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Point3> {
  gtsam::Point3 measured_field_;
  gtsam::Point3 map_field_ref_;
  gtsam::Rot3 target_R_sensor_;

 public:
  /**
   * @brief Constructs the factor, caching the sensor rotation.
   * @param pose_key GTSAM key for the AUV pose.
   * @param bias_key GTSAM key for the magnetometer hard-iron bias (shared across the mission).
   * @param measured_field The measured magnetic field vector (sensor frame) [T].
   * @param reference_field The reference magnetic field vector (map frame) [T].
   * @param target_T_sensor The static transformation from target to sensor.
   * @param noise_model The noise model for the measurement.
   */
  MagCalibFactorArm(gtsam::Key pose_key, gtsam::Key bias_key, const gtsam::Point3& measured_field,
                    const gtsam::Point3& reference_field, const gtsam::Pose3& target_T_sensor,
                    const gtsam::SharedNoiseModel& noise_model)
      : NoiseModelFactor2<gtsam::Pose3, gtsam::Point3>(noise_model, pose_key, bias_key),
        measured_field_(measured_field),
        map_field_ref_(reference_field),
        target_R_sensor_(target_T_sensor.rotation()) {}

  /**
   * @brief Evaluates the error and Jacobians for the factor.
   * @param pose The AUV pose estimate.
   * @param bias The hard-iron bias estimate (sensor frame).
   * @param H_pose Optional Jacobian matrix with respect to pose.
   * @param H_bias Optional Jacobian matrix with respect to bias.
   * @return The 3D error vector (predicted - measured).
   */
  gtsam::Vector evaluateError(const gtsam::Pose3& pose, const gtsam::Point3& bias,
                              gtsam::OptionalMatrixType H_pose = nullptr,
                              gtsam::OptionalMatrixType H_bias = nullptr) const override {
    gtsam::Matrix33 H_unrotate_target = gtsam::Matrix33::Zero();
    gtsam::Point3 predicted_field_target =
        pose.rotation().unrotate(map_field_ref_, H_pose ? &H_unrotate_target : nullptr);
    gtsam::Point3 predicted_field = target_R_sensor_.unrotate(predicted_field_target);

    // 3D magnetic field residual, with the hard-iron offset added to the prediction
    gtsam::Vector3 error = predicted_field + bias - measured_field_;

    if (H_pose) {
      // Jacobian with respect to pose (3x6)
      H_pose->setZero(3, 6);
      H_pose->block<3, 3>(0, 0) = target_R_sensor_.transpose() * H_unrotate_target;
    }

    if (H_bias) {
      // Jacobian with respect to bias (3x3)
      *H_bias = gtsam::I_3x3;
    }

    return error;
  }
};

}  // namespace coug_fgo::factors
