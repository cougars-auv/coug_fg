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

class DepthOriginDeltaFactorArm : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3> {
  double measured_depth_;
  gtsam::Point3 target_p_sensor_;

 public:
  DepthOriginDeltaFactorArm(gtsam::Key delta_key, gtsam::Key pose_key, double measured_depth,
                            const gtsam::Pose3& target_T_sensor,
                            const gtsam::SharedNoiseModel& noise_model)
      : NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>(noise_model, delta_key, pose_key),
        measured_depth_(measured_depth),
        target_p_sensor_(target_T_sensor.translation()) {}

  auto evaluateError(const gtsam::Pose3& delta, const gtsam::Pose3& pose,
                     gtsam::OptionalMatrixType H_delta = nullptr,
                     gtsam::OptionalMatrixType H_pose = nullptr) const -> gtsam::Vector override {
    // Transform the agent's pose into the map frame with the origin delta
    gtsam::Matrix66 H_compose_delta = gtsam::Matrix66::Zero();
    gtsam::Matrix66 H_compose_pose = gtsam::Matrix66::Zero();
    const gtsam::Pose3 map_T_agent = delta.compose(pose, H_delta ? &H_compose_delta : nullptr,
                                                   H_pose ? &H_compose_pose : nullptr);

    gtsam::Matrix36 H_transform = gtsam::Matrix36::Zero();
    gtsam::Point3 predicted_position =
        map_T_agent.transformFrom(target_p_sensor_, (H_delta || H_pose) ? &H_transform : nullptr);

    // 1D depth residual
    const double error = predicted_position.z() - measured_depth_;

    if (H_delta) {
      // Jacobian with respect to delta (1x6)
      *H_delta = H_transform.row(2) * H_compose_delta;
    }

    if (H_pose) {
      // Jacobian with respect to pose (1x6)
      *H_pose = H_transform.row(2) * H_compose_pose;
    }

    return gtsam::Vector1(error);
  }
};

}  // namespace coug_fg::factors
