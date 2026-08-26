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
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace coug_fg::factors {

class MagCalibFactorArm : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Point3> {
  gtsam::Point3 measured_field_;
  gtsam::Point3 map_field_ref_;
  gtsam::Rot3 target_R_sensor_;

 public:
  MagCalibFactorArm(gtsam::Key pose_key, gtsam::Key bias_key, const gtsam::Point3& measured_field,
                    const gtsam::Point3& reference_field, const gtsam::Pose3& target_T_sensor,
                    const gtsam::SharedNoiseModel& noise_model)
      : NoiseModelFactor2<gtsam::Pose3, gtsam::Point3>(noise_model, pose_key, bias_key),
        measured_field_(measured_field),
        map_field_ref_(reference_field),
        target_R_sensor_(target_T_sensor.rotation()) {}

  gtsam::Vector evaluateError(const gtsam::Pose3& pose, const gtsam::Point3& bias,
                              gtsam::OptionalMatrixType H_pose = nullptr,
                              gtsam::OptionalMatrixType H_bias = nullptr) const override {
    gtsam::Matrix33 H_unrotate_R = gtsam::Matrix33::Zero();
    gtsam::Point3 target_field =
        pose.rotation().unrotate(map_field_ref_, H_pose ? &H_unrotate_R : nullptr);
    gtsam::Point3 predicted_field = target_R_sensor_.unrotate(target_field);

    // 3D magnetic field residual, with the hard-iron offset added to the prediction
    gtsam::Vector3 error = predicted_field + bias - measured_field_;

    if (H_pose) {
      // Jacobian with respect to pose (3x6)
      H_pose->setZero(3, 6);
      H_pose->block<3, 3>(0, 0) = target_R_sensor_.transpose() * H_unrotate_R;
    }

    if (H_bias) {
      // Jacobian with respect to bias (3x3)
      *H_bias = gtsam::I_3x3;
    }

    return error;
  }
};

}  // namespace coug_fg::factors
