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
 * @file ahrs_origin_delta_factor.hpp
 * @brief GTSAM factor for AHRS attitude measurements with an origin delta.
 * @author Nelson Durrant
 * @date May 2026
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace coug_fgo::factors {

/**
 * @class AhrsOriginDeltaFactorArm
 * @brief GTSAM factor for AHRS attitude measurements with an origin delta.
 */
class AhrsOriginDeltaFactorArm : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3> {
  gtsam::Rot3 measured_orientation_;
  gtsam::Rot3 target_R_sensor_;

 public:
  /**
   * @brief References a measured map-frame orientation to true north.
   * @param measured_orientation The measured orientation of the sensor in the map frame [rad].
   * @param mag_declination East-positive magnetic declination (NOAA convention) [rad].
   * @return The measured orientation referenced to true north [rad].
   */
  static gtsam::Rot3 trueNorthOrientation(const gtsam::Rot3& measured_orientation,
                                          double mag_declination) {
    return gtsam::Rot3::Yaw(-mag_declination) * measured_orientation;
  }

  /**
   * @brief Conjugates a map-frame orientation covariance into the sensor-frame tangent space.
   * @param map_covariance The orientation covariance about the map-frame axes [rad^2].
   * @param measured_orientation The measured orientation of the sensor in the map frame [rad].
   * @return The equivalent covariance in the sensor-frame tangent space [rad^2].
   */
  static gtsam::Matrix3 sensorTangentCovariance(const gtsam::Matrix3& map_covariance,
                                                const gtsam::Rot3& measured_orientation) {
    const gtsam::Matrix3 map_R_sensor = measured_orientation.matrix();

    return map_R_sensor.transpose() * map_covariance * map_R_sensor;
  }

  /**
   * @brief Constructs the factor.
   * @param delta_key GTSAM key for the agent's origin delta (agent frame to map frame).
   * @param pose_key GTSAM key for the AUV pose, in the agent's own frame.
   * @param measured_orientation The measured orientation of the sensor in the map frame [rad].
   * @param target_T_sensor The static transformation from target to sensor.
   * @param mag_declination East-positive magnetic declination (NOAA convention) [rad].
   * @param noise_model The noise model for the measurement (sensor-frame tangent space).
   */
  AhrsOriginDeltaFactorArm(gtsam::Key delta_key, gtsam::Key pose_key,
                           const gtsam::Rot3& measured_orientation,
                           const gtsam::Pose3& target_T_sensor, double mag_declination,
                           const gtsam::SharedNoiseModel& noise_model)
      : NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>(noise_model, delta_key, pose_key),
        measured_orientation_(trueNorthOrientation(measured_orientation, mag_declination)),
        target_R_sensor_(target_T_sensor.rotation()) {}

  /**
   * @brief Evaluates the error and Jacobians for the factor.
   * @param delta The agent's origin delta estimate.
   * @param pose The AUV pose estimate, in the agent's own frame.
   * @param H_delta Optional Jacobian matrix with respect to delta.
   * @param H_pose Optional Jacobian matrix with respect to pose.
   * @return The 3D orientation residual (sensor-frame tangent space) [rad].
   */
  gtsam::Vector evaluateError(const gtsam::Pose3& delta, const gtsam::Pose3& pose,
                              gtsam::OptionalMatrixType H_delta = nullptr,
                              gtsam::OptionalMatrixType H_pose = nullptr) const override {
    const bool need_jacobians = H_delta || H_pose;

    // Transform the agent's pose into the map frame with the origin delta
    gtsam::Matrix66 H_compose_delta = gtsam::Matrix66::Zero();
    gtsam::Matrix66 H_compose_pose = gtsam::Matrix66::Zero();
    gtsam::Pose3 map_T_agent = delta.compose(pose, H_delta ? &H_compose_delta : nullptr,
                                             H_pose ? &H_compose_pose : nullptr);

    gtsam::Matrix36 H_rotation = gtsam::Matrix36::Zero();
    gtsam::Rot3 map_R_agent = map_T_agent.rotation(need_jacobians ? &H_rotation : nullptr);

    gtsam::Matrix33 H_compose = gtsam::Matrix33::Zero();
    gtsam::Rot3 predicted_orientation =
        map_R_agent.compose(target_R_sensor_, need_jacobians ? &H_compose : nullptr);

    // 3D orientation residual (Lie algebra)
    gtsam::Matrix33 H_between = gtsam::Matrix33::Zero();
    gtsam::Rot3 orientation_error = measured_orientation_.between(
        predicted_orientation, nullptr, need_jacobians ? &H_between : nullptr);

    gtsam::Matrix33 H_logmap = gtsam::Matrix33::Zero();
    gtsam::Vector3 error =
        gtsam::Rot3::Logmap(orientation_error, need_jacobians ? &H_logmap : nullptr);

    if (need_jacobians) {
      const gtsam::Matrix36 H_agent = H_logmap * H_between * H_compose * H_rotation;

      if (H_delta) {
        // Jacobian with respect to delta (3x6)
        *H_delta = H_agent * H_compose_delta;
      }

      if (H_pose) {
        // Jacobian with respect to pose (3x6)
        *H_pose = H_agent * H_compose_pose;
      }
    }

    return error;
  }
};

}  // namespace coug_fgo::factors
