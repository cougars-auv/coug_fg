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

class DvlTightPreintegrator {
 public:
  DvlTightPreintegrator() { reset(); }

  void reset() {
    measured_translation_ = gtsam::Vector3::Zero();
    translation_cov_ = gtsam::Matrix3::Zero();
    J_p_bg_ = gtsam::Matrix3::Zero();
    cross_cov_rot_trans_ = gtsam::Matrix3::Zero();
    prev_delta_R_ik_ = gtsam::Rot3();
  }

  void integrateMeasurement(const gtsam::Vector3& measured_velocity, const gtsam::Rot3& delta_R_ik,
                            const gtsam::Rot3& target_R_dvl, double dt,
                            const gtsam::Matrix3& measured_velocity_cov,
                            const gtsam::Matrix3& rot_cov_k, const gtsam::Matrix3& J_bg_k) {
    // Transform the rotation error from the previous frame to this one
    const gtsam::Matrix3 k_R_prev = (prev_delta_R_ik_.inverse() * delta_R_ik).matrix().transpose();
    cross_cov_rot_trans_ = k_R_prev * cross_cov_rot_trans_;

    // Transform the velocity into the anchor frame (i) and integrate
    const gtsam::Vector3 target_v_dvl = target_R_dvl.rotate(measured_velocity);
    const gtsam::Vector3 i_v_dvl = delta_R_ik.rotate(target_v_dvl);
    measured_translation_ += i_v_dvl * dt;

    gtsam::Matrix3 J_vel = delta_R_ik.matrix() * target_R_dvl.matrix() * dt;
    gtsam::Matrix3 J_rot = -delta_R_ik.matrix() * gtsam::skewSymmetric(target_v_dvl) * dt;
    translation_cov_ += (J_vel * measured_velocity_cov * J_vel.transpose()) +
                        (J_rot * rot_cov_k * J_rot.transpose()) + (J_rot * cross_cov_rot_trans_) +
                        (J_rot * cross_cov_rot_trans_).transpose();
    cross_cov_rot_trans_ += rot_cov_k * J_rot.transpose();

    J_p_bg_ += J_rot * J_bg_k;
    prev_delta_R_ik_ = delta_R_ik;
  }

  [[nodiscard]] auto delta() const -> gtsam::Vector3 { return measured_translation_; }

  [[nodiscard]] auto covariance() const -> gtsam::Matrix3 { return translation_cov_; }

  [[nodiscard]] auto preintMeasDerivativeWrtBias() const -> gtsam::Matrix3 { return J_p_bg_; }

 private:
  gtsam::Vector3 measured_translation_;
  gtsam::Matrix3 translation_cov_;
  gtsam::Matrix3 J_p_bg_;
  gtsam::Matrix3 cross_cov_rot_trans_;
  gtsam::Rot3 prev_delta_R_ik_;
};

}  // namespace coug_fg::utils
