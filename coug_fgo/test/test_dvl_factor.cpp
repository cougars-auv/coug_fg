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
 * @file test_dvl_factor.cpp
 * @brief Unit tests for dvl_factor.hpp.
 * @author Nelson Durrant (w Claude Opus 5)
 * @date May 2026
 */

#include <gtest/gtest.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/factorTesting.h>

#include "coug_fgo/factors/dvl_factor.hpp"

namespace {

constexpr double kStep = 1e-5;  // finite difference step
constexpr double kJacobianTol = 1e-5;
constexpr double kResidualTol = 1e-9;

}  // namespace

/**
 * @brief Verify Jacobians against numerical differentiation.
 */
TEST(DvlFactorArmTest, Jacobians) {
  gtsam::Key pose_key = gtsam::symbol_shorthand::X(1);
  gtsam::Key vel_key = gtsam::symbol_shorthand::V(1);
  gtsam::Key bias_key = gtsam::symbol_shorthand::B(1);
  gtsam::SharedNoiseModel model = gtsam::noiseModel::Isotropic::Sigma(3, 0.1);
  gtsam::Pose3 target_T_sensor(gtsam::Rot3::Ypr(0.1, -0.1, 0.1), gtsam::Point3(0.5, 0.5, 0.5));
  gtsam::Pose3 target_T_imu(gtsam::Rot3::Ypr(-0.2, 0.1, 0.3), gtsam::Point3(0.1, 0.2, 0.3));
  gtsam::Vector3 measured_vel(1.0, 0.5, -0.2);
  gtsam::Vector3 measured_gyro(0.1, -0.3, 0.2);

  coug_fgo::factors::DvlFactorArm factor(pose_key, vel_key, bias_key, target_T_sensor, target_T_imu,
                                         measured_vel, measured_gyro, model);

  gtsam::Values values;
  values.insert(pose_key,
                gtsam::Pose3(gtsam::Rot3::Ypr(0.1, 0.2, 0.3), gtsam::Point3(1.0, 2.0, 4.0)));
  values.insert(vel_key, gtsam::Vector3(1.5, -0.5, 0.2));
  values.insert(bias_key, gtsam::imuBias::ConstantBias(gtsam::Vector3(0.01, -0.02, 0.03),
                                                       gtsam::Vector3(0.02, -0.01, 0.01)));

  EXPECT_TRUE(
      gtsam::internal::testFactorJacobians("DvlFactorArm", factor, values, kStep, kJacobianTol));
}

/**
 * @brief Verify the residual against an independently predicted measurement.
 */
TEST(DvlFactorArmTest, Residual) {
  gtsam::Key pose_key = gtsam::symbol_shorthand::X(1);
  gtsam::Key vel_key = gtsam::symbol_shorthand::V(1);
  gtsam::Key bias_key = gtsam::symbol_shorthand::B(1);
  gtsam::SharedNoiseModel model = gtsam::noiseModel::Isotropic::Sigma(3, 0.1);
  gtsam::Pose3 target_T_sensor(gtsam::Rot3::Ypr(0.1, -0.1, 0.1), gtsam::Point3(0.5, 0.5, 0.5));
  gtsam::Pose3 target_T_imu(gtsam::Rot3::Ypr(-0.2, 0.1, 0.3), gtsam::Point3(0.1, 0.2, 0.3));
  gtsam::Vector3 measured_gyro(0.1, -0.3, 0.2);

  gtsam::Pose3 pose(gtsam::Rot3::Ypr(0.1, 0.2, 0.3), gtsam::Point3(1.0, 2.0, 4.0));
  gtsam::Vector3 vel_map(1.5, -0.5, 0.2);
  gtsam::imuBias::ConstantBias bias(gtsam::Vector3(0.01, -0.02, 0.03),
                                    gtsam::Vector3(0.02, -0.01, 0.01));

  // Velocity the DVL would report: target motion plus lever arm rotation
  const gtsam::Vector3 omega_target =
      target_T_imu.rotation().matrix() * (measured_gyro - bias.gyroscope());
  const gtsam::Vector3 vel_target = pose.rotation().matrix().transpose() * vel_map;
  const gtsam::Vector3 lever_arm_vel = omega_target.cross(target_T_sensor.translation());
  const gtsam::Vector3 sensor_vel =
      target_T_sensor.rotation().matrix().transpose() * (vel_target + lever_arm_vel);

  // Measured short of the prediction
  const gtsam::Vector3 offset(0.01, -0.02, 0.03);
  coug_fgo::factors::DvlFactorArm factor(pose_key, vel_key, bias_key, target_T_sensor, target_T_imu,
                                         sensor_vel - offset, measured_gyro, model);

  const gtsam::Vector expected = offset;
  EXPECT_TRUE(
      gtsam::assert_equal(expected, factor.evaluateError(pose, vel_map, bias), kResidualTol));
}
