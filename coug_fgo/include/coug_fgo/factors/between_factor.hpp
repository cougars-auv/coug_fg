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
 * @file between_factor.hpp
 * @brief GTSAM between factor given pose measurements of baselink.
 * @author Kalliyan Velasco
 * @date July 2026
 */

#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace coug_fgo::factors {

class BetweenFactorArm : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3> {
 private:
  gtsam::Pose3 measured_between_;
  gtsam::Pose3 target_T_base_;

 public:
  BetweenFactorArm(gtsam::Key key1, gtsam::Key key2, const gtsam::Pose3& measured_between,
                   const gtsam::Pose3& target_T_base, const gtsam::SharedNoiseModel& noise_model)
      : NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>(noise_model, key1, key2),
        measured_between_(measured_between),
        target_T_base_(target_T_base) {}

  gtsam::Vector evaluateError(const gtsam::Pose3& pose1, const gtsam::Pose3& pose2,
                              gtsam::OptionalMatrixType H1 = nullptr,
                              gtsam::OptionalMatrixType H2 = nullptr) const override {
    gtsam::Matrix66 Hcompose1, Hcompose2;
    gtsam::Pose3 base1 = pose1.compose(target_T_base_, H1 ? &Hcompose1 : nullptr);
    gtsam::Pose3 base2 = pose2.compose(target_T_base_, H2 ? &Hcompose2 : nullptr);

    gtsam::Matrix66 Hbetween1, Hbetween2;
    gtsam::Pose3 predicted_between =
        base1.between(base2, H1 ? &Hbetween1 : nullptr, H2 ? &Hbetween2 : nullptr);

    if (H1) *H1 = Hbetween1 * Hcompose1;

    if (H2) *H2 = Hbetween2 * Hcompose2;

    return measured_between_.localCoordinates(predicted_between);
  }
};

}  // namespace coug_fgo::factors
