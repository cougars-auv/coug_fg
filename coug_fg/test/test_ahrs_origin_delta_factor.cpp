// Copyright 2026 BYU FROST Lab
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
#include <gtsam/base/Vector.h>
#include <gtsam/base/types.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/factorTesting.h>

#include "coug_fg/factors/ahrs_origin_delta_factor.hpp"

namespace {

using coug_fg::factors::AhrsOriginDeltaFactorArm;

using gtsam::symbol_shorthand::O;  // Neighbor origin delta Pose3 (x,y,z,r,p,y)
using gtsam::symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)

constexpr double kStep = 1e-5;  // finite difference step
constexpr double kJacobianTol = 1e-5;
constexpr double kResidualTol = 1e-9;

}  // namespace

TEST(AhrsOriginDeltaFactorArmTest, Jacobians) {
  const gtsam::Key delta_key = O(1);
  const gtsam::Key pose_key = X(1);
  const gtsam::SharedNoiseModel model = gtsam::noiseModel::Isotropic::Sigma(3, 0.1);
  const gtsam::Pose3 target_T_sensor(gtsam::Rot3::Ypr(0.1, -0.1, 0.1), gtsam::Point3::Zero());
  const gtsam::Rot3 measured_attitude = gtsam::Rot3::Ypr(0.5, 0.1, -0.1);
  const double magnetic_declination = 0.05;

  const AhrsOriginDeltaFactorArm factor(delta_key, pose_key, measured_attitude, target_T_sensor,
                                        magnetic_declination, model);

  gtsam::Values values;
  values.insert(delta_key,
                gtsam::Pose3(gtsam::Rot3::Ypr(0.5, 0.05, -0.05), gtsam::Point3(3.0, -2.0, 0.5)));
  values.insert(pose_key,
                gtsam::Pose3(gtsam::Rot3::Ypr(0.1, 0.2, 0.3), gtsam::Point3(1.0, 2.0, 4.0)));

  EXPECT_TRUE(gtsam::internal::testFactorJacobians("AhrsOriginDeltaFactorArm", factor, values,
                                                   kStep, kJacobianTol));
}

TEST(AhrsOriginDeltaFactorArmTest, Residual) {
  const gtsam::Key delta_key = O(1);
  const gtsam::Key pose_key = X(1);
  const gtsam::SharedNoiseModel model = gtsam::noiseModel::Isotropic::Sigma(3, 0.1);
  const gtsam::Pose3 target_T_sensor(gtsam::Rot3::Ypr(0.1, -0.1, 0.1), gtsam::Point3::Zero());
  const gtsam::Pose3 delta(gtsam::Rot3::Ypr(0.5, 0.05, -0.05), gtsam::Point3(3.0, -2.0, 0.5));
  const gtsam::Pose3 pose(gtsam::Rot3::Ypr(0.1, 0.2, 0.3), gtsam::Point3(1.0, 2.0, 4.0));
  const double magnetic_declination = 0.05;

  // The pose is in the agent's own frame, so the origin delta places it in the map frame
  const gtsam::Rot3 map_R_sensor = (delta * pose).rotation() * target_T_sensor.rotation();

  // Turn the measurement off the state, then back to magnetic north
  const gtsam::Vector3 offset(0.02, -0.03, 0.05);
  const gtsam::Rot3 measured_attitude =
      gtsam::Rot3::Yaw(magnetic_declination) * map_R_sensor * gtsam::Rot3::Expmap(offset);

  const AhrsOriginDeltaFactorArm factor(delta_key, pose_key, measured_attitude, target_T_sensor,
                                        magnetic_declination, model);

  // Logmap(Expmap(-offset)) undoes the injected turn
  const gtsam::Vector expected = -offset;
  EXPECT_TRUE(gtsam::assert_equal(expected, factor.evaluateError(delta, pose), kResidualTol));
}
