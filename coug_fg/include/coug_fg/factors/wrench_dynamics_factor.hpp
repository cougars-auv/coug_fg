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

namespace coug_fg::factors {

class WrenchDynamicsFactorArm
    : public gtsam::NoiseModelFactor4<gtsam::Pose3, gtsam::Vector3, gtsam::Pose3, gtsam::Vector3> {
  double dt_;
  gtsam::Vector3 target_force_;
  gtsam::Matrix33 mass_;
  gtsam::Matrix33 linear_drag_;
  gtsam::Matrix33 quad_drag_;
  gtsam::Matrix33 mass_inv_;

 public:
  WrenchDynamicsFactorArm(gtsam::Key pose_key_i, gtsam::Key vel_key_i, gtsam::Key pose_key_j,
                          gtsam::Key vel_key_j, double dt, gtsam::Vector3 const& control_force,
                          gtsam::Pose3 const& target_T_sensor, gtsam::Matrix33 const& mass,
                          gtsam::Matrix33 const& linear_drag, gtsam::Matrix33 const& quad_drag,
                          gtsam::SharedNoiseModel const& noise_model)
      : NoiseModelFactor4<gtsam::Pose3, gtsam::Vector3, gtsam::Pose3, gtsam::Vector3>(
            noise_model, pose_key_i, vel_key_i, pose_key_j, vel_key_j),
        dt_(dt),
        target_force_(target_T_sensor.rotation().rotate(control_force)),
        mass_(mass),
        linear_drag_(linear_drag),
        quad_drag_(quad_drag),
        mass_inv_(mass.inverse()) {}

  auto evaluateError(gtsam::Pose3 const& pose_i, gtsam::Vector3 const& vel_i,
                     gtsam::Pose3 const& pose_j, gtsam::Vector3 const& vel_j,
                     gtsam::OptionalMatrixType H_pose_i = nullptr,
                     gtsam::OptionalMatrixType H_vel_i = nullptr,
                     gtsam::OptionalMatrixType H_pose_j = nullptr,
                     gtsam::OptionalMatrixType H_vel_j = nullptr) const -> gtsam::Vector override {
    gtsam::Matrix33 H_unrotate_Ri = gtsam::Matrix33::Zero();
    gtsam::Matrix33 H_unrotate_vi = gtsam::Matrix33::Zero();
    gtsam::Matrix33 H_unrotate_Rj = gtsam::Matrix33::Zero();
    gtsam::Matrix33 H_unrotate_vj = gtsam::Matrix33::Zero();
    gtsam::Vector3 const target_v_i = pose_i.rotation().unrotate(
        vel_i, H_pose_i ? &H_unrotate_Ri : nullptr, H_vel_i ? &H_unrotate_vi : nullptr);
    gtsam::Vector3 const target_v_j = pose_j.rotation().unrotate(
        vel_j, H_pose_j ? &H_unrotate_Rj : nullptr, H_vel_j ? &H_unrotate_vj : nullptr);

    gtsam::Vector3 const target_v_i_abs = target_v_i.cwiseAbs();
    gtsam::Vector3 const drag_force =
        -(linear_drag_ * target_v_i + quad_drag_ * target_v_i.cwiseProduct(target_v_i_abs));

    // Jacobian of the velocity prediction with respect to target_v_i
    gtsam::Matrix33 J_scale = gtsam::Matrix33::Zero();
    if (H_pose_i || H_vel_i) {
      gtsam::Matrix33 const J_drag_v =
          -(linear_drag_ + 2.0 * quad_drag_ * target_v_i_abs.asDiagonal());
      J_scale = gtsam::Matrix33::Identity() + dt_ * mass_inv_ * J_drag_v;
    }

    gtsam::Vector3 const target_accel = mass_inv_ * (target_force_ + drag_force);
    gtsam::Vector3 const target_v_pred = target_v_i + target_accel * dt_;

    // 3D velocity difference residual
    gtsam::Vector3 const error = target_v_j - target_v_pred;

    if (H_pose_i) {
      // Jacobian with respect to pose_i (3x6)
      H_pose_i->setZero(3, 6);
      H_pose_i->block<3, 3>(0, 0) = -J_scale * H_unrotate_Ri;
    }

    if (H_vel_i) {
      // Jacobian with respect to vel_i (3x3)
      *H_vel_i = -J_scale * H_unrotate_vi;
    }

    if (H_pose_j) {
      // Jacobian with respect to pose_j (3x6)
      H_pose_j->setZero(3, 6);
      H_pose_j->block<3, 3>(0, 0) = H_unrotate_Rj;
    }

    if (H_vel_j) {
      // Jacobian with respect to vel_j (3x3)
      *H_vel_j = H_unrotate_vj;
    }

    return error;
  }
};

}  // namespace coug_fg::factors
