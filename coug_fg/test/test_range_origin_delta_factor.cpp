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

#include <gtest/gtest.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/factorTesting.h>

#include "coug_fg/factors/range_origin_delta_factor.hpp"

namespace {

constexpr double kStep = 1e-5;  // finite difference step
constexpr double kJacobianTol = 1e-5;
constexpr double kResidualTol = 1e-9;

}  // namespace

TEST(RangeOriginDeltaFactorArmTest, Jacobians) {
  gtsam::Key pose_key_l = gtsam::symbol_shorthand::X(1);
  gtsam::Key delta_key_n = gtsam::symbol_shorthand::O(1);
  gtsam::Key pose_key_n = gtsam::symbol_shorthand::X(2);
  gtsam::SharedNoiseModel model = gtsam::noiseModel::Isotropic::Sigma(1, 0.1);
  gtsam::Pose3 target_T_sensor_l(gtsam::Rot3::Ypr(0.1, -0.1, 0.1), gtsam::Point3(0.5, 0.5, 0.5));
  gtsam::Pose3 target_T_sensor_n(gtsam::Rot3::Ypr(-0.2, 0.1, 0.3), gtsam::Point3(0.2, -0.4, 0.1));
  double measured_range = 10.0;

  coug_fg::factors::RangeOriginDeltaFactorArm factor(pose_key_l, delta_key_n, pose_key_n,
                                                     measured_range, target_T_sensor_l,
                                                     target_T_sensor_n, model);

  gtsam::Values values;
  values.insert(pose_key_l,
                gtsam::Pose3(gtsam::Rot3::Ypr(0.1, 0.2, 0.3), gtsam::Point3(1.0, 2.0, 4.0)));
  values.insert(delta_key_n,
                gtsam::Pose3(gtsam::Rot3::Ypr(0.5, 0.05, -0.05), gtsam::Point3(3.0, -2.0, 0.5)));
  values.insert(pose_key_n,
                gtsam::Pose3(gtsam::Rot3::Ypr(0.4, -0.1, 0.2), gtsam::Point3(8.0, 5.0, 6.0)));

  EXPECT_TRUE(gtsam::internal::testFactorJacobians("RangeOriginDeltaFactorArm", factor, values,
                                                   kStep, kJacobianTol));
}

TEST(RangeOriginDeltaFactorArmTest, Residual) {
  gtsam::Key pose_key_l = gtsam::symbol_shorthand::X(1);
  gtsam::Key delta_key_n = gtsam::symbol_shorthand::O(1);
  gtsam::Key pose_key_n = gtsam::symbol_shorthand::X(2);
  gtsam::SharedNoiseModel model = gtsam::noiseModel::Isotropic::Sigma(1, 0.1);
  gtsam::Pose3 target_T_sensor_l(gtsam::Rot3::Ypr(0.1, -0.1, 0.1), gtsam::Point3(0.5, 0.5, 0.5));
  gtsam::Pose3 target_T_sensor_n(gtsam::Rot3::Ypr(-0.2, 0.1, 0.3), gtsam::Point3(0.2, -0.4, 0.1));
  gtsam::Pose3 pose_l(gtsam::Rot3::Ypr(0.1, 0.2, 0.3), gtsam::Point3(1.0, 2.0, 4.0));
  gtsam::Pose3 delta_n(gtsam::Rot3::Ypr(0.5, 0.05, -0.05), gtsam::Point3(3.0, -2.0, 0.5));
  gtsam::Pose3 pose_n(gtsam::Rot3::Ypr(0.4, -0.1, 0.2), gtsam::Point3(8.0, 5.0, 6.0));

  // The neighbor's pose is in its own frame, so the origin delta places it in the map frame
  const gtsam::Pose3 map_T_n = delta_n * pose_n;

  // Separation of the two modems, not of the two targets
  const gtsam::Point3 map_p_sensor_l =
      pose_l.rotation().matrix() * target_T_sensor_l.translation() + pose_l.translation();
  const gtsam::Point3 map_p_sensor_n =
      map_T_n.rotation().matrix() * target_T_sensor_n.translation() + map_T_n.translation();
  const double sensor_range = (map_p_sensor_l - map_p_sensor_n).norm();

  // Measured short of the prediction
  constexpr double kOffset = 0.4;
  coug_fg::factors::RangeOriginDeltaFactorArm factor(pose_key_l, delta_key_n, pose_key_n,
                                                     sensor_range - kOffset, target_T_sensor_l,
                                                     target_T_sensor_n, model);

  EXPECT_NEAR(factor.evaluateError(pose_l, delta_n, pose_n)(0), kOffset, kResidualTol);
}
