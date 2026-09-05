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

class MagFactorArm : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
  gtsam::Point3 measured_field_;
  gtsam::Point3 map_field_ref_;
  gtsam::Rot3 target_R_sensor_;

 public:
  MagFactorArm(gtsam::Key pose_key, gtsam::Point3 const& measured_field,
               gtsam::Point3 const& reference_field, gtsam::Pose3 const& target_T_sensor,
               gtsam::SharedNoiseModel const& noise_model)
      : NoiseModelFactor1<gtsam::Pose3>(noise_model, pose_key),
        measured_field_(measured_field),
        map_field_ref_(reference_field),
        target_R_sensor_(target_T_sensor.rotation()) {}

  auto evaluateError(gtsam::Pose3 const& pose, gtsam::OptionalMatrixType H = nullptr) const
      -> gtsam::Vector override {
    gtsam::Matrix33 H_unrotate_R = gtsam::Matrix33::Zero();
    gtsam::Point3 const target_field =
        pose.rotation().unrotate(map_field_ref_, H ? &H_unrotate_R : nullptr);
    gtsam::Point3 const predicted_field = target_R_sensor_.unrotate(target_field);

    // 3D magnetic field residual
    gtsam::Vector3 const error = predicted_field - measured_field_;

    if (H) {
      // Jacobian with respect to pose (3x6)
      H->setZero(3, 6);
      H->block<3, 3>(0, 0) = target_R_sensor_.transpose() * H_unrotate_R;
    }

    return error;
  }
};

}  // namespace coug_fg::factors
