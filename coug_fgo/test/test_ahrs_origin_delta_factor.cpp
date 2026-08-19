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
 * @file test_ahrs_origin_delta_factor.cpp
 * @brief Unit tests for ahrs_origin_delta_factor.hpp.
 * @author Nelson Durrant (w Claude Opus 5)
 * @date May 2026
 */

#include <gtest/gtest.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/factorTesting.h>

#include "coug_fgo/factors/ahrs_origin_delta_factor.hpp"

namespace {

constexpr double kStep = 1e-5;  // finite difference step
constexpr double kJacobianTol = 1e-5;
constexpr double kResidualTol = 1e-9;

}  // namespace

/**
 * @brief Verify Jacobians against numerical differentiation.
 */
TEST(AhrsOriginDeltaFactorArmTest, Jacobians) {
  gtsam::Key delta_key = gtsam::symbol_shorthand::O(1);
  gtsam::Key pose_key = gtsam::symbol_shorthand::X(1);
  gtsam::SharedNoiseModel model = gtsam::noiseModel::Isotropic::Sigma(3, 0.1);
  gtsam::Pose3 target_T_sensor(gtsam::Rot3::Ypr(0.1, -0.1, 0.1), gtsam::Point3::Zero());
  gtsam::Rot3 measured_attitude = gtsam::Rot3::Ypr(0.5, 0.1, -0.1);
  double magnetic_declination = 0.05;

  coug_fgo::factors::AhrsOriginDeltaFactorArm factor(delta_key, pose_key, measured_attitude,
                                                     target_T_sensor, magnetic_declination, model);

  gtsam::Values values;
  values.insert(delta_key,
                gtsam::Pose3(gtsam::Rot3::Ypr(0.5, 0.05, -0.05), gtsam::Point3(3.0, -2.0, 0.5)));
  values.insert(pose_key,
                gtsam::Pose3(gtsam::Rot3::Ypr(0.1, 0.2, 0.3), gtsam::Point3(1.0, 2.0, 4.0)));

  EXPECT_TRUE(gtsam::internal::testFactorJacobians("AhrsOriginDeltaFactorArm", factor, values,
                                                   kStep, kJacobianTol));
}

/**
 * @brief Verify the residual against an independently predicted measurement.
 */
TEST(AhrsOriginDeltaFactorArmTest, Residual) {
  gtsam::Key delta_key = gtsam::symbol_shorthand::O(1);
  gtsam::Key pose_key = gtsam::symbol_shorthand::X(1);
  gtsam::SharedNoiseModel model = gtsam::noiseModel::Isotropic::Sigma(3, 0.1);
  gtsam::Pose3 target_T_sensor(gtsam::Rot3::Ypr(0.1, -0.1, 0.1), gtsam::Point3::Zero());
  gtsam::Pose3 delta(gtsam::Rot3::Ypr(0.5, 0.05, -0.05), gtsam::Point3(3.0, -2.0, 0.5));
  gtsam::Pose3 pose(gtsam::Rot3::Ypr(0.1, 0.2, 0.3), gtsam::Point3(1.0, 2.0, 4.0));
  double magnetic_declination = 0.05;

  // The pose is in the agent's own frame, so the origin delta places it in the map frame
  const gtsam::Rot3 map_R_sensor = (delta * pose).rotation() * target_T_sensor.rotation();

  // Turn the measurement off the state, then back to magnetic north
  const gtsam::Vector3 offset(0.02, -0.03, 0.05);
  const gtsam::Rot3 measured_attitude =
      gtsam::Rot3::Yaw(magnetic_declination) * map_R_sensor * gtsam::Rot3::Expmap(offset);

  coug_fgo::factors::AhrsOriginDeltaFactorArm factor(delta_key, pose_key, measured_attitude,
                                                     target_T_sensor, magnetic_declination, model);

  // Logmap(Expmap(-offset)) undoes the injected turn
  const gtsam::Vector expected = -offset;
  EXPECT_TRUE(gtsam::assert_equal(expected, factor.evaluateError(delta, pose), kResidualTol));
}
