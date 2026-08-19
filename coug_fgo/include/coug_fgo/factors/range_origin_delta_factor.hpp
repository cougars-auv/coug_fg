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
 * @file range_origin_delta_factor.hpp
 * @brief GTSAM factor for acoustic range measurements between two AUVs with an origin delta.
 * @author Kalliyan Velasco & Nelson Durrant
 * @date August 2026
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace coug_fgo::factors {

/**
 * @class RangeOriginDeltaFactorArm
 * @brief GTSAM factor for acoustic range measurements between two AUVs with an origin delta.
 */
class RangeOriginDeltaFactorArm
    : public gtsam::NoiseModelFactor3<gtsam::Pose3, gtsam::Pose3, gtsam::Pose3> {
  double measured_range_;
  gtsam::Point3 target_p_sensor_l_;
  gtsam::Point3 target_p_sensor_n_;

 public:
  /**
   * @brief Constructs the factor.
   * @param pose_key_l GTSAM key for the local AUV pose.
   * @param delta_key_n GTSAM key for the neighbor's origin delta (neighbor frame to map frame).
   * @param pose_key_n GTSAM key for the neighbor AUV pose, in the neighbor's own frame.
   * @param measured_range The measured range between both modem sensors [m].
   * @param target_T_sensor_l The static transformation from local target to local sensor.
   * @param target_T_sensor_n The static transformation from neighbor target to neighbor sensor.
   * @param noise_model The noise model for the measurement.
   */
  RangeOriginDeltaFactorArm(gtsam::Key pose_key_l, gtsam::Key delta_key_n, gtsam::Key pose_key_n,
                            const double measured_range, const gtsam::Pose3& target_T_sensor_l,
                            const gtsam::Pose3& target_T_sensor_n,
                            const gtsam::SharedNoiseModel& noise_model)
      : gtsam::NoiseModelFactor3<gtsam::Pose3, gtsam::Pose3, gtsam::Pose3>(noise_model, pose_key_l,
                                                                           delta_key_n, pose_key_n),
        measured_range_(measured_range),
        target_p_sensor_l_(target_T_sensor_l.translation()),
        target_p_sensor_n_(target_T_sensor_n.translation()) {}

  /**
   * @brief Evaluates the error and Jacobians for the factor.
   * @param pose_l The local AUV pose estimate.
   * @param delta_n The neighbor's origin delta estimate.
   * @param pose_n The neighbor AUV pose estimate, in the neighbor's own frame.
   * @param H_pose_l Optional Jacobian matrix with respect to pose_l.
   * @param H_delta_n Optional Jacobian matrix with respect to delta_n.
   * @param H_pose_n Optional Jacobian matrix with respect to pose_n.
   * @return The 1D range residual [m].
   */
  gtsam::Vector evaluateError(const gtsam::Pose3& pose_l, const gtsam::Pose3& delta_n,
                              const gtsam::Pose3& pose_n,
                              gtsam::OptionalMatrixType H_pose_l = nullptr,
                              gtsam::OptionalMatrixType H_delta_n = nullptr,
                              gtsam::OptionalMatrixType H_pose_n = nullptr) const override {
    gtsam::Matrix36 H_transform_l = gtsam::Matrix36::Zero();
    gtsam::Point3 map_p_sensor_l =
        pose_l.transformFrom(target_p_sensor_l_, H_pose_l ? &H_transform_l : nullptr);

    // Transform the neighbor's pose into the map frame with the origin delta
    gtsam::Matrix66 H_compose_delta = gtsam::Matrix66::Zero();
    gtsam::Matrix66 H_compose_pose = gtsam::Matrix66::Zero();
    gtsam::Pose3 map_T_n = delta_n.compose(pose_n, H_delta_n ? &H_compose_delta : nullptr,
                                           H_pose_n ? &H_compose_pose : nullptr);

    gtsam::Matrix36 H_transform_n = gtsam::Matrix36::Zero();
    gtsam::Point3 map_p_sensor_n = map_T_n.transformFrom(
        target_p_sensor_n_, (H_delta_n || H_pose_n) ? &H_transform_n : nullptr);

    gtsam::Matrix13 H_distance_l = gtsam::Matrix13::Zero();
    gtsam::Matrix13 H_distance_n = gtsam::Matrix13::Zero();
    double predicted_range =
        gtsam::distance3(map_p_sensor_l, map_p_sensor_n, H_pose_l ? &H_distance_l : nullptr,
                         (H_delta_n || H_pose_n) ? &H_distance_n : nullptr);

    // 1D range residual
    double error = predicted_range - measured_range_;

    if (H_pose_l) {
      // Jacobian with respect to pose_l (1x6)
      *H_pose_l = H_distance_l * H_transform_l;
    }

    if (H_delta_n) {
      // Jacobian with respect to delta_n (1x6)
      *H_delta_n = H_distance_n * H_transform_n * H_compose_delta;
    }

    if (H_pose_n) {
      // Jacobian with respect to pose_n (1x6)
      *H_pose_n = H_distance_n * H_transform_n * H_compose_pose;
    }

    return gtsam::Vector1(error);
  }
};

}  // namespace coug_fgo::factors
