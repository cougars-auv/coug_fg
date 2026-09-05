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
#include <gtsam/geometry/Rot3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <cmath>

#include "coug_fg/factors/ahrs_factor.hpp"

namespace coug_fg::factors {

class AhrsYawFactorArm : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
  double measured_yaw_;
  gtsam::Rot3 target_R_sensor_;

 public:
  AhrsYawFactorArm(gtsam::Key pose_key, gtsam::Rot3 const& measured_orientation,
                   gtsam::Pose3 const& target_T_sensor, double mag_declination,
                   gtsam::SharedNoiseModel const& noise_model)
      : NoiseModelFactor1<gtsam::Pose3>(noise_model, pose_key),
        measured_yaw_(
            AhrsFactorArm::trueNorthOrientation(measured_orientation, mag_declination).yaw()),
        target_R_sensor_(target_T_sensor.rotation()) {}

  auto evaluateError(gtsam::Pose3 const& pose, gtsam::OptionalMatrixType H = nullptr) const
      -> gtsam::Vector override {
    gtsam::Matrix33 H_compose = gtsam::Matrix33::Zero();
    gtsam::Rot3 const predicted_orientation =
        pose.rotation().compose(target_R_sensor_, H ? &H_compose : nullptr);

    // Singular at +/-90 deg pitch (gimbal lock)
    gtsam::Matrix13 H_yaw = gtsam::Matrix13::Zero();
    double const predicted_yaw = predicted_orientation.yaw(H ? &H_yaw : nullptr);

    // 1D heading residual, wrapped into [-pi, pi]
    double const error = std::remainder(predicted_yaw - measured_yaw_, 2.0 * M_PI);

    if (H) {
      // Jacobian with respect to pose (1x6)
      H->setZero(1, 6);
      H->block<1, 3>(0, 0) = H_yaw * H_compose;
    }

    return gtsam::Vector1(error);
  }
};

}  // namespace coug_fg::factors
