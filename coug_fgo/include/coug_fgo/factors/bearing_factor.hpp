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
 * @file bearing_factor.hpp
 * @brief GTSAM factor for acoustic azimuth/elevation measurements between two
 *        agents, each with a static sensor lever arm relative to its own pose.
 * @author Kalliyan Velasco
 * @date July 2026
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Unit3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <functional>

namespace coug_fgo::factors {

/**
 * @class BearingFactorArm
 * @brief GTSAM factor for acoustic bearing (azimuth, elevation) measurements
 *        between two agents, each offset from its own pose by a static
 *        sensor lever arm.
 *
 * The measurement is taken by the LOCAL agent's sensor and points toward the
 * NEIGHBOR agent's sensor. Both agents' poses are estimated in the graph, so
 * this is the two-pose / inter-agent generalization of the single-landmark
 * 3D bearing factor from Real et al. 2025 ("Modular Acoustic Graph SLAM for
 * Underwater Monitoring With Autonomous Underwater Vehicles"), Sec. III-E.
 *
 * Like the paper, the error is computed as a Unit3 tangent-space difference
 * rather than a raw (azimuth, elevation) subtraction, avoiding the ±π
 * discontinuity inherent to angle differencing.
 */
class BearingFactorArm : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3> {
  gtsam::Point2 measured_azi_el_;
  gtsam::Pose3 target_T_sensor_l_;
  gtsam::Pose3 target_T_sensor_n_;

 public:
  /**
   * @brief Constructs the factor, caching the sensor lever arms.
   * @param pose_key_l GTSAM key for the local AUV pose.
   * @param pose_key_n GTSAM key for the neighbor AUV pose.
   * @param measured_azi_el The measured azimuth, elevation between both modem sensors (rad).
   * @param target_T_sensor_l The static transformation from local target to local sensor.
   * @param target_T_sensor_n The static transformation from neighbor target to neighbor sensor.
   * @param noise_model The noise model for the measurement (1D, on the range residual).
   */
  BearingFactorArm(gtsam::Key pose_key_l, gtsam::Key pose_key_n,
                   const gtsam::Point2& measured_azi_el, const gtsam::Pose3& target_T_sensor_l,
                   const gtsam::Pose3& target_T_sensor_n,
                   const gtsam::SharedNoiseModel& noise_model)
      : gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>(noise_model, pose_key_l, pose_key_n),
        measured_azi_el_(measured_azi_el),
        target_T_sensor_l_(target_T_sensor_l),
        target_T_sensor_n_(target_T_sensor_n) {}

  /**
   * @brief Evaluates the error and Jacobians for the factor.
   * @param pose_l The local AUV target pose estimate.
   * @param pose_n The neighbor AUV target pose estimate.
   * @param H_pose_l Optional 2x6 Jacobian wrt the local pose's tangent.
   * @param H_pose_n Optional 2x6 Jacobian wrt the neighbor pose's tangent.
   * @return The 2D error (predicted - measured).
   */
  gtsam::Vector evaluateError(const gtsam::Pose3& pose_l, const gtsam::Pose3& pose_n,
                              gtsam::OptionalMatrixType H_pose_l = nullptr,
                              gtsam::OptionalMatrixType H_pose_n = nullptr) const override {
    // Factor graph poses are target poses in world, want sensor pose in world
    gtsam::Matrix66 H_sensor_l_pose;
    const gtsam::Pose3 sensor_l_world =
        pose_l.compose(target_T_sensor_l_, H_pose_l ? &H_sensor_l_pose : nullptr, nullptr);

    gtsam::Matrix66 H_sensor_n_pose;
    const gtsam::Pose3 sensor_n_world =
        pose_n.compose(target_T_sensor_n_, H_pose_n ? &H_sensor_n_pose : nullptr, nullptr);

    // Predicted bearing unit vector. TODO: Double check gtsam documentation. Correct bearing we
    // want?
    gtsam::Matrix26 H_pred_sensor_l;
    gtsam::Matrix23 H_pred_point;

    const gtsam::Unit3 predicted =
        sensor_l_world.bearing(sensor_n_world.translation(), H_pose_l ? &H_pred_sensor_l : nullptr,
                               H_pose_n ? &H_pred_point : nullptr);

    // Measured bearing unit vector
    const double az = measured_azi_el_(0);
    const double el = measured_azi_el_(1);

    const gtsam::Point3 los_meas(std::cos(el) * std::cos(az), std::cos(el) * std::sin(az),
                                 std::sin(el));

    const gtsam::Unit3 measured(los_meas);

    // Error between unit vectors. Tangent plane has two dimensions, so vector size 2 (not because
    // azimuth elevation).
    gtsam::Matrix22 H_error_pred;
    const gtsam::Vector2 error =
        -predicted.errorVector(measured, (H_pose_l || H_pose_n) ? &H_error_pred : nullptr, nullptr);

    // Chain rule
    if (H_pose_l) {
      *H_pose_l = -H_error_pred * H_pred_sensor_l * H_sensor_l_pose;
    }

    if (H_pose_n) {
      gtsam::Matrix36 H_translation;
      sensor_n_world.translation(H_translation);

      *H_pose_n = -H_error_pred * H_pred_point * H_translation * H_sensor_n_pose;
    }

    return error;
  }
};

}  // namespace coug_fgo::factors
