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

#include "coug_fgo/factors/depth_origin_delta_factor.hpp"

namespace {

constexpr double kStep = 1e-5;  // finite difference step
constexpr double kJacobianTol = 1e-5;
constexpr double kResidualTol = 1e-9;

}  // namespace

TEST(DepthOriginDeltaFactorArmTest, Jacobians) {
  gtsam::Key delta_key = gtsam::symbol_shorthand::O(1);
  gtsam::Key pose_key = gtsam::symbol_shorthand::X(1);
  gtsam::SharedNoiseModel model = gtsam::noiseModel::Isotropic::Sigma(1, 0.1);
  gtsam::Pose3 target_T_sensor(gtsam::Rot3::Ypr(0.1, -0.1, 0.1), gtsam::Point3(0.5, 0.5, 0.5));
  double measured_depth = 5.0;

  coug_fgo::factors::DepthOriginDeltaFactorArm factor(delta_key, pose_key, measured_depth,
                                                      target_T_sensor, model);

  gtsam::Values values;
  values.insert(delta_key,
                gtsam::Pose3(gtsam::Rot3::Ypr(0.5, 0.05, -0.05), gtsam::Point3(3.0, -2.0, 0.5)));
  values.insert(pose_key,
                gtsam::Pose3(gtsam::Rot3::Ypr(0.1, 0.2, 0.3), gtsam::Point3(1.0, 2.0, 4.0)));

  EXPECT_TRUE(gtsam::internal::testFactorJacobians("DepthOriginDeltaFactorArm", factor, values,
                                                   kStep, kJacobianTol));
}

TEST(DepthOriginDeltaFactorArmTest, Residual) {
  gtsam::Key delta_key = gtsam::symbol_shorthand::O(1);
  gtsam::Key pose_key = gtsam::symbol_shorthand::X(1);
  gtsam::SharedNoiseModel model = gtsam::noiseModel::Isotropic::Sigma(1, 0.1);
  gtsam::Pose3 target_T_sensor(gtsam::Rot3::Ypr(0.1, -0.1, 0.1), gtsam::Point3(0.5, 0.5, 0.5));
  gtsam::Pose3 delta(gtsam::Rot3::Ypr(0.5, 0.05, -0.05), gtsam::Point3(3.0, -2.0, 0.5));
  gtsam::Pose3 pose(gtsam::Rot3::Ypr(0.1, 0.2, 0.3), gtsam::Point3(1.0, 2.0, 4.0));

  // The pose is in the agent's own frame, so the origin delta places it in the map frame
  const gtsam::Pose3 map_T_agent = delta * pose;

  // Depth of the sensor itself, not of the target
  const double sensor_depth =
      (map_T_agent.rotation().matrix() * target_T_sensor.translation() + map_T_agent.translation())
          .z();

  // Measured shallower than the state predicts
  constexpr double kOffset = 0.25;
  coug_fgo::factors::DepthOriginDeltaFactorArm factor(delta_key, pose_key, sensor_depth - kOffset,
                                                      target_T_sensor, model);

  EXPECT_NEAR(factor.evaluateError(delta, pose)(0), kOffset, kResidualTol);
}
