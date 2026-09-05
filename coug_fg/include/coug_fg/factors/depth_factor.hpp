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

class DepthFactorArm : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
  double measured_depth_;
  gtsam::Point3 target_p_sensor_;

 public:
  DepthFactorArm(gtsam::Key pose_key, double measured_depth, gtsam::Pose3 const& target_T_sensor,
                 gtsam::SharedNoiseModel const& noise_model)
      : NoiseModelFactor1<gtsam::Pose3>(noise_model, pose_key),
        measured_depth_(measured_depth),
        target_p_sensor_(target_T_sensor.translation()) {}

  auto evaluateError(gtsam::Pose3 const& pose, gtsam::OptionalMatrixType H = nullptr) const
      -> gtsam::Vector override {
    gtsam::Matrix36 H_transform = gtsam::Matrix36::Zero();
    gtsam::Point3 predicted_position =
        pose.transformFrom(target_p_sensor_, H ? &H_transform : nullptr);

    // 1D depth residual
    double const error = predicted_position.z() - measured_depth_;

    if (H) {
      // Jacobian with respect to pose (1x6)
      *H = H_transform.row(2);
    }

    return gtsam::Vector1(error);
  }
};

}  // namespace coug_fg::factors
