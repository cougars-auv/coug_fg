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
 * @file test_range_factor.cpp
 * @brief Unit tests for range_factor.hpp.
 * @author Nelson Durrant (w Opus 5)
 * @date August 2026
 */

#include <gtest/gtest.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>

#include <functional>
#include <optional>

#include "coug_fgo/factors/range_factor.hpp"

/**
 * @brief Verify Jacobians against numerical differentiation.
 */
TEST(RangeFactorArmTest, Jacobians) {
  gtsam::Key pose_key_l = gtsam::symbol_shorthand::X(1);
  gtsam::Key pose_key_n = gtsam::symbol_shorthand::X(2);
  gtsam::SharedNoiseModel model = gtsam::noiseModel::Isotropic::Sigma(1, 0.1);
  gtsam::Pose3 target_T_sensor_l(gtsam::Rot3::Ypr(0.1, -0.1, 0.1), gtsam::Point3(0.5, 0.5, 0.5));
  gtsam::Pose3 target_T_sensor_n(gtsam::Rot3::Ypr(-0.2, 0.1, 0.3), gtsam::Point3(0.2, -0.4, 0.1));
  double measured_range = 10.0;

  coug_fgo::factors::RangeFactorArm factor(pose_key_l, pose_key_n, measured_range,
                                           target_T_sensor_l, target_T_sensor_n, model);

  gtsam::Pose3 pose_l(gtsam::Rot3::Ypr(0.1, 0.2, 0.3), gtsam::Point3(1.0, 2.0, 4.0));
  gtsam::Pose3 pose_n(gtsam::Rot3::Ypr(0.4, -0.1, 0.2), gtsam::Point3(8.0, 5.0, 6.0));

  auto evalFunc = [&](const gtsam::Pose3& pl, const gtsam::Pose3& pn) {
    return factor.evaluateError(pl, pn, nullptr, nullptr);
  };

  gtsam::Matrix expectedH_pose_l =
      gtsam::numericalDerivative21<gtsam::Vector, gtsam::Pose3, gtsam::Pose3>(evalFunc, pose_l,
                                                                              pose_n, 1e-5);

  gtsam::Matrix expectedH_pose_n =
      gtsam::numericalDerivative22<gtsam::Vector, gtsam::Pose3, gtsam::Pose3>(evalFunc, pose_l,
                                                                              pose_n, 1e-5);

  gtsam::Matrix actualH_pose_l, actualH_pose_n;
  factor.evaluateError(pose_l, pose_n, &actualH_pose_l, &actualH_pose_n);

  EXPECT_TRUE(gtsam::assert_equal(expectedH_pose_l, actualH_pose_l, 1e-5));
  EXPECT_TRUE(gtsam::assert_equal(expectedH_pose_n, actualH_pose_n, 1e-5));
  EXPECT_EQ(actualH_pose_l.rows(), 1);
  EXPECT_EQ(actualH_pose_l.cols(), 6);
  EXPECT_EQ(actualH_pose_n.rows(), 1);
  EXPECT_EQ(actualH_pose_n.cols(), 6);
}
