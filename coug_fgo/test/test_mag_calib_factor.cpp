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
 * @file test_mag_calib_factor.cpp
 * @brief Unit tests for mag_calib_factor.hpp.
 * @author Nelson Durrant (w Opus 5)
 * @date August 2026
 */

#include <gtest/gtest.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>

#include <functional>
#include <optional>

#include "coug_fgo/factors/mag_calib_factor.hpp"

/**
 * @brief Verify Jacobians against numerical differentiation.
 */
TEST(MagCalibFactorArmTest, Jacobians) {
  gtsam::Key poseKey = gtsam::symbol_shorthand::X(1);
  gtsam::Key biasKey = gtsam::symbol_shorthand::M(0);
  gtsam::SharedNoiseModel model = gtsam::noiseModel::Isotropic::Sigma(3, 0.1);
  gtsam::Pose3 target_T_sensor(gtsam::Rot3::Ypr(0.1, -0.1, 0.1), gtsam::Point3::Zero());
  gtsam::Vector3 reference_field(3.9634e-06, 2.08423e-05, -4.57678e-05);
  gtsam::Vector3 measured_field(4.1000e-06, 2.00000e-05, -4.50000e-05);

  coug_fgo::factors::MagCalibFactorArm factor(poseKey, biasKey, measured_field, reference_field,
                                              target_T_sensor, model);

  gtsam::Pose3 pose(gtsam::Rot3::Ypr(0.1, 0.2, 0.3), gtsam::Point3(1.0, 2.0, 4.0));
  gtsam::Point3 bias(-3.3e-06, 6.5e-07, 1.16e-05);

  auto evalFunc = [&](const gtsam::Pose3& p, const gtsam::Point3& b) {
    return factor.evaluateError(p, b, nullptr, nullptr);
  };

  gtsam::Matrix expectedH1 =
      gtsam::numericalDerivative21<gtsam::Vector, gtsam::Pose3, gtsam::Point3>(evalFunc, pose, bias,
                                                                               1e-5);
  gtsam::Matrix expectedH2 =
      gtsam::numericalDerivative22<gtsam::Vector, gtsam::Pose3, gtsam::Point3>(evalFunc, pose, bias,
                                                                               1e-5);

  gtsam::Matrix actualH1, actualH2;
  factor.evaluateError(pose, bias, &actualH1, &actualH2);

  EXPECT_TRUE(gtsam::assert_equal(expectedH1, actualH1, 1e-11));
  EXPECT_TRUE(gtsam::assert_equal(expectedH2, actualH2, 1e-11));
  EXPECT_EQ(actualH1.rows(), 3);
  EXPECT_EQ(actualH1.cols(), 6);
}
