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
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Unit3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <cmath>

namespace coug_fg::factors {

// Two-pose generalization of the single-landmark 3D bearing factor in Real et al. 2025, "Modular
// Acoustic Graph SLAM for Underwater Monitoring With Autonomous Underwater Vehicles", Sec. III-E
class BearingOriginDeltaFactorArm
    : public gtsam::NoiseModelFactor3<gtsam::Pose3, gtsam::Pose3, gtsam::Pose3> {
  gtsam::Point2 measured_azi_el_;
  gtsam::Pose3 target_T_sensor_l_;
  gtsam::Pose3 target_T_sensor_n_;

 public:
  static auto losDirection(const gtsam::Point2& azi_el) -> gtsam::Unit3 {
    const double azimuth = azi_el(0);
    const double elevation = azi_el(1);

    return gtsam::Unit3(gtsam::Point3(std::cos(elevation) * std::cos(azimuth),
                                      std::cos(elevation) * std::sin(azimuth),
                                      std::sin(elevation)));
  }

  static auto unit3TangentCovariance(const gtsam::Matrix22& azi_el_covariance,
                                     const gtsam::Point2& azi_el) -> gtsam::Matrix22 {
    const double azimuth = azi_el(0);
    const double elevation = azi_el(1);

    // Columns of d(direction)/d(azimuth, elevation), tangent to the unit sphere
    gtsam::Matrix32 J_direction_azi_el = gtsam::Matrix32::Zero();
    J_direction_azi_el.col(0) << -std::cos(elevation) * std::sin(azimuth),
        std::cos(elevation) * std::cos(azimuth), 0.0;
    J_direction_azi_el.col(1) << -std::sin(elevation) * std::cos(azimuth),
        -std::sin(elevation) * std::sin(azimuth), std::cos(elevation);

    const gtsam::Unit3 measured_direction = losDirection(azi_el);
    const gtsam::Matrix22 J_basis_azi_el =
        measured_direction.basis().transpose() * J_direction_azi_el;

    // Conjugate azimuth/elevation covariance into the Unit3 tangent space
    gtsam::Matrix22 tangent_covariance =
        J_basis_azi_el * azi_el_covariance * J_basis_azi_el.transpose();

    // Isotropic floor, azimuth carries no direction information at the poles
    constexpr double kMinSigma = 1.0e-3;  // [rad]
    tangent_covariance += (kMinSigma * kMinSigma) * gtsam::Matrix22::Identity();

    return tangent_covariance;
  }

  BearingOriginDeltaFactorArm(gtsam::Key pose_key_l, gtsam::Key delta_key_n, gtsam::Key pose_key_n,
                              const gtsam::Point2& measured_azi_el,
                              const gtsam::Pose3& target_T_sensor_l,
                              const gtsam::Pose3& target_T_sensor_n,
                              const gtsam::SharedNoiseModel& noise_model)
      : gtsam::NoiseModelFactor3<gtsam::Pose3, gtsam::Pose3, gtsam::Pose3>(noise_model, pose_key_l,
                                                                           delta_key_n, pose_key_n),
        measured_azi_el_(measured_azi_el),
        target_T_sensor_l_(target_T_sensor_l),
        target_T_sensor_n_(target_T_sensor_n) {}

  auto evaluateError(const gtsam::Pose3& pose_l, const gtsam::Pose3& delta_n,
                     const gtsam::Pose3& pose_n, gtsam::OptionalMatrixType H_pose_l = nullptr,
                     gtsam::OptionalMatrixType H_delta_n = nullptr,
                     gtsam::OptionalMatrixType H_pose_n = nullptr) const -> gtsam::Vector override {
    gtsam::Matrix66 H_compose_l = gtsam::Matrix66::Zero();
    const gtsam::Pose3 map_T_sensor_l =
        pose_l.compose(target_T_sensor_l_, H_pose_l ? &H_compose_l : nullptr);

    // Transform the neighbor's pose into the map frame with the origin delta
    gtsam::Matrix66 H_compose_delta = gtsam::Matrix66::Zero();
    gtsam::Matrix66 H_compose_pose = gtsam::Matrix66::Zero();
    const gtsam::Pose3 map_T_n = delta_n.compose(pose_n, H_delta_n ? &H_compose_delta : nullptr,
                                                 H_pose_n ? &H_compose_pose : nullptr);

    gtsam::Matrix66 H_compose_n = gtsam::Matrix66::Zero();
    const gtsam::Pose3 map_T_sensor_n =
        map_T_n.compose(target_T_sensor_n_, (H_delta_n || H_pose_n) ? &H_compose_n : nullptr);

    gtsam::Matrix26 H_bearing_l = gtsam::Matrix26::Zero();
    gtsam::Matrix23 H_bearing_n = gtsam::Matrix23::Zero();
    const gtsam::Unit3 predicted_direction =
        map_T_sensor_l.bearing(map_T_sensor_n.translation(), H_pose_l ? &H_bearing_l : nullptr,
                               (H_delta_n || H_pose_n) ? &H_bearing_n : nullptr);

    // 2D bearing residual, anchored at the measured direction to match the noise model basis
    const gtsam::Unit3 measured_direction = losDirection(measured_azi_el_);
    gtsam::Matrix22 H_error = gtsam::Matrix22::Zero();
    const gtsam::Vector2 error = measured_direction.errorVector(
        predicted_direction, nullptr, (H_pose_l || H_delta_n || H_pose_n) ? &H_error : nullptr);

    if (H_pose_l) {
      // Jacobian with respect to pose_l (2x6)
      *H_pose_l = H_error * H_bearing_l * H_compose_l;
    }

    if (H_delta_n || H_pose_n) {
      gtsam::Matrix36 H_translation_n = gtsam::Matrix36::Zero();
      map_T_sensor_n.translation(H_translation_n);
      const gtsam::Matrix26 H_sensor_n = H_error * H_bearing_n * H_translation_n * H_compose_n;

      if (H_delta_n) {
        // Jacobian with respect to delta_n (2x6)
        *H_delta_n = H_sensor_n * H_compose_delta;
      }

      if (H_pose_n) {
        // Jacobian with respect to pose_n (2x6)
        *H_pose_n = H_sensor_n * H_compose_pose;
      }
    }

    return error;
  }
};

}  // namespace coug_fg::factors
