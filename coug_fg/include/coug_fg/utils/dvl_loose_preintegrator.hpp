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
#include <gtsam/geometry/Rot3.h>

namespace coug_fg::utils {

class DvlLoosePreintegrator {
 public:
  DvlLoosePreintegrator() { reset(gtsam::Rot3()); }

  void reset(const gtsam::Rot3& initial_orientation,
             const gtsam::Rot3& target_R_ahrs = gtsam::Rot3(),
             const gtsam::Rot3& target_R_dvl = gtsam::Rot3(),
             const gtsam::Matrix3& ahrs_tangent_cov = gtsam::Matrix3::Zero()) {
    map_R_i_ = initial_orientation;
    target_R_ahrs_ = target_R_ahrs.matrix();
    dvl_R_ahrs_ = (target_R_dvl.inverse() * target_R_ahrs).matrix();
    ahrs_tangent_cov_ = ahrs_tangent_cov;
    measured_translation_ = gtsam::Vector3::Zero();
    velocity_noise_cov_ = gtsam::Matrix3::Zero();
    J_ahrs_ = gtsam::Matrix3::Zero();
  }

  void integrateMeasurement(const gtsam::Vector3& measured_velocity,
                            const gtsam::Rot3& measured_orientation, double dt,
                            const gtsam::Matrix3& measured_velocity_cov) {
    // Transform the velocity into the anchor frame (i) and integrate
    const gtsam::Rot3 i_R_k = map_R_i_.between(measured_orientation);
    const gtsam::Vector3 i_v_dvl = i_R_k.rotate(measured_velocity);
    measured_translation_ += i_v_dvl * dt;

    gtsam::Matrix3 J_vel = i_R_k.matrix() * dt;
    velocity_noise_cov_ += J_vel * measured_velocity_cov * J_vel.transpose();

    J_ahrs_ += dt * (gtsam::skewSymmetric(i_v_dvl) * target_R_ahrs_ -
                     i_R_k.matrix() * gtsam::skewSymmetric(measured_velocity) * dvl_R_ahrs_);
  }

  [[nodiscard]] auto delta() const -> gtsam::Vector3 { return measured_translation_; }

  [[nodiscard]] auto covariance() const -> gtsam::Matrix3 {
    return velocity_noise_cov_ + J_ahrs_ * ahrs_tangent_cov_ * J_ahrs_.transpose();
  }

 private:
  gtsam::Rot3 map_R_i_;
  gtsam::Matrix3 target_R_ahrs_;
  gtsam::Matrix3 dvl_R_ahrs_;
  gtsam::Matrix3 ahrs_tangent_cov_;
  gtsam::Vector3 measured_translation_;
  gtsam::Matrix3 velocity_noise_cov_;
  gtsam::Matrix3 J_ahrs_;
};

}  // namespace coug_fg::utils
