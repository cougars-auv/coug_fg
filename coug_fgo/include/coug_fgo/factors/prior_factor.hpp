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
 * @file prior_factor.hpp
 * @brief GTSAM factor for prior given pose of baselink.
 * @author Kalliyan Velasco
 * @date July 2026
 */

#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace coug_fgo::factors {

class PriorFactorArm : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
 private:
  gtsam::Pose3 measured_pose_;    // world_T_baselink
  gtsam::Pose3 target_T_sensor_;  // target_T_baselink

 public:
  PriorFactorArm(gtsam::Key pose_key, const gtsam::Pose3& measured_pose,
                 const gtsam::Pose3& target_T_sensor, const gtsam::SharedNoiseModel& noise_model)
      : NoiseModelFactor1<gtsam::Pose3>(noise_model, pose_key),
        measured_pose_(measured_pose),
        target_T_sensor_(target_T_sensor) {}

  gtsam::Vector evaluateError(const gtsam::Pose3& pose,
                              gtsam::OptionalMatrixType H = nullptr) const override {
    gtsam::Matrix66 H_comp;
    gtsam::Pose3 predicted_pose = pose.compose(target_T_sensor_, H ? &H_comp : nullptr);

    if (H) {
      *H = H_comp;
    }

    return measured_pose_.localCoordinates(predicted_pose);
  }
};

}  // namespace coug_fgo::factors
