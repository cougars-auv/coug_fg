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
 * @file range_factor.hpp
 * @brief GTSAM factor for a scalar acoustic range measurement between two sensors, each with a
 * lever arm.
 * @author Kalliyan Velasco
 * @date July 2026
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace coug_fgo::factors {

/**
 * @class RangeFactorArm
 * @brief GTSAM factor for a scalar acoustic range measurement between two sensors, each with a
 * lever arm.
 */
class RangeFactorArm : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3> {
  double measured_range_;
  gtsam::Point3 target_p_sensor_l_;
  gtsam::Point3 target_p_sensor_n_;

 public:
  /**
   * @brief Constructs the factor, caching the sensor lever arms.
   * @param pose_key_l GTSAM key for the local AUV pose.
   * @param pose_key_n GTSAM key for the neighbor AUV pose.
   * @param measured_range The measured range between both modem sensors (m).
   * @param target_T_sensor_l The static transformation from local target to local sensor.
   * @param target_T_sensor_n The static transformation from neighbor target to neighbor sensor.
   * @param noise_model The noise model for the measurement (1D, on the range residual).
   */
  RangeFactorArm(gtsam::Key pose_key_l, gtsam::Key pose_key_n, const double measured_range,
                 const gtsam::Pose3& target_T_sensor_l, const gtsam::Pose3& target_T_sensor_n,
                 const gtsam::SharedNoiseModel& noise_model)
      : gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>(noise_model, pose_key_l, pose_key_n),
        measured_range_(measured_range),
        target_p_sensor_l_(target_T_sensor_l.translation()),
        target_p_sensor_n_(target_T_sensor_n.translation()) {}

  /**
   * @brief Evaluates the error and Jacobians for the factor.
   * @param pose_l The local AUV target pose estimate.
   * @param pose_n The neighbor AUV target pose estimate.
   * @param H_pose_l_target Optional 1x6 Jacobian wrt the local pose's tangent.
   * @param H_pose_n_target Optional 1x6 Jacobian wrt the neighbor pose's tangent.
   * @return The 1D range error (predicted - measured).
   */
  gtsam::Vector evaluateError(const gtsam::Pose3& pose_l, const gtsam::Pose3& pose_n,
                              gtsam::OptionalMatrixType H_pose_l_target = nullptr,
                              gtsam::OptionalMatrixType H_pose_n_target = nullptr) const override {
    // Factor graph poses are target poses in world, want sensor pose in world
    gtsam::Matrix H_sensor_l_wrt_pose_l, H_sensor_n_wrt_pose_n;
    gtsam::Point3 sensor_l_world = pose_l.transformFrom(
        target_p_sensor_l_, H_pose_l_target ? &H_sensor_l_wrt_pose_l : nullptr);
    gtsam::Point3 sensor_n_world = pose_n.transformFrom(
        target_p_sensor_n_, H_pose_n_target ? &H_sensor_n_wrt_pose_n : nullptr);

    // Range between the two sensor points in world frame, with Jacobians wrt each point (1x3).
    gtsam::Matrix H_range_wrt_sensor_l, H_range_wrt_sensor_n;
    double predicted_range = gtsam::distance3(sensor_l_world, sensor_n_world,
                                              H_pose_l_target ? &H_range_wrt_sensor_l : nullptr,
                                              H_pose_n_target ? &H_range_wrt_sensor_n : nullptr);

    // Chain rule: d(range)/d(pose tangent) = d(range)/d(sensor_point) * d(sensor_point)/d(pose
    // tangent)
    if (H_pose_l_target) {
      *H_pose_l_target = H_range_wrt_sensor_l * H_sensor_l_wrt_pose_l;
    }
    if (H_pose_n_target) {
      *H_pose_n_target = H_range_wrt_sensor_n * H_sensor_n_wrt_pose_n;
    }

    return gtsam::Vector1(predicted_range - measured_range_);
  }
};

}  // namespace coug_fgo::factors
