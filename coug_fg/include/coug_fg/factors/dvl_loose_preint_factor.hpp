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
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace coug_fg::factors {

class DvlLoosePreintFactorArm : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3> {
  gtsam::Point3 target_p_sensor_;
  gtsam::Vector3 measured_translation_;

 public:
  DvlLoosePreintFactorArm(gtsam::Key pose_key_i, gtsam::Key pose_key_j,
                          const gtsam::Pose3& target_T_sensor,
                          const gtsam::Vector3& measured_translation,
                          const gtsam::SharedNoiseModel& noise_model)
      : gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>(noise_model, pose_key_i, pose_key_j),
        target_p_sensor_(target_T_sensor.translation()),
        measured_translation_(measured_translation) {}

  auto evaluateError(const gtsam::Pose3& pose_i, const gtsam::Pose3& pose_j,
                     gtsam::OptionalMatrixType H_pose_i = nullptr,
                     gtsam::OptionalMatrixType H_pose_j = nullptr) const -> gtsam::Vector override {
    gtsam::Matrix36 H_transform_from_j = gtsam::Matrix36::Zero();
    const gtsam::Point3 map_p_sensor_j =
        pose_j.transformFrom(target_p_sensor_, H_pose_j ? &H_transform_from_j : nullptr);

    gtsam::Matrix36 H_transform_to_i = gtsam::Matrix36::Zero();
    gtsam::Matrix33 H_transform_to_j = gtsam::Matrix33::Zero();
    const gtsam::Point3 i_p_sensor_j =
        pose_i.transformTo(map_p_sensor_j, H_pose_i ? &H_transform_to_i : nullptr,
                           H_pose_j ? &H_transform_to_j : nullptr);

    const gtsam::Vector3 predicted_translation = i_p_sensor_j - target_p_sensor_;

    // 3D translation residual
    const gtsam::Vector3 error = predicted_translation - measured_translation_;

    if (H_pose_i) {
      // Jacobian with respect to pose_i (3x6)
      *H_pose_i = H_transform_to_i;
    }

    if (H_pose_j) {
      // Jacobian with respect to pose_j (3x6)
      *H_pose_j = H_transform_to_j * H_transform_from_j;
    }

    return error;
  }
};

}  // namespace coug_fg::factors
