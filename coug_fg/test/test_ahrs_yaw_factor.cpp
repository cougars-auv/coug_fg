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
#include <gtsam/base/types.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/factorTesting.h>

#include <cmath>

#include "coug_fg/factors/ahrs_yaw_factor.hpp"

namespace {

using coug_fg::factors::AhrsYawFactorArm;

using gtsam::symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)

constexpr double kStep = 1e-5;  // finite difference step
constexpr double kJacobianTol = 1e-5;
constexpr double kResidualTol = 1e-9;

}  // namespace

TEST(AhrsYawFactorArmTest, Jacobians) {
  const gtsam::Key pose_key = X(1);
  const gtsam::SharedNoiseModel model = gtsam::noiseModel::Isotropic::Sigma(1, 0.1);
  const gtsam::Pose3 target_T_sensor(gtsam::Rot3::Ypr(0.1, -0.1, 0.1), gtsam::Point3::Zero());
  const gtsam::Rot3 measured_attitude = gtsam::Rot3::Ypr(0.5, 0.1, -0.1);
  const double magnetic_declination = 0.05;

  const AhrsYawFactorArm factor(pose_key, measured_attitude, target_T_sensor, magnetic_declination,
                                model);

  gtsam::Values values;
  values.insert(pose_key,
                gtsam::Pose3(gtsam::Rot3::Ypr(0.1, 0.2, 0.3), gtsam::Point3(1.0, 2.0, 4.0)));

  EXPECT_TRUE(gtsam::internal::testFactorJacobians("AhrsYawFactorArm", factor, values, kStep,
                                                   kJacobianTol));
}

TEST(AhrsYawFactorArmTest, Residual) {
  const gtsam::Key pose_key = X(1);
  const gtsam::SharedNoiseModel model = gtsam::noiseModel::Isotropic::Sigma(1, 0.1);
  const gtsam::Pose3 target_T_sensor(gtsam::Rot3::Ypr(0.1, -0.1, 0.1), gtsam::Point3::Zero());
  const gtsam::Pose3 pose(gtsam::Rot3::Ypr(0.1, 0.2, 0.3), gtsam::Point3(1.0, 2.0, 4.0));
  const double magnetic_declination = 0.05;

  // Heading the sensor holds in the map frame
  const double sensor_yaw = (pose.rotation() * target_T_sensor.rotation()).yaw();

  // Measured short of the state's heading
  constexpr double kOffset = 0.1;
  const gtsam::Rot3 measured_attitude =
      gtsam::Rot3::Yaw(magnetic_declination) * gtsam::Rot3::Yaw(sensor_yaw - kOffset);

  const AhrsYawFactorArm factor(pose_key, measured_attitude, target_T_sensor, magnetic_declination,
                                model);

  EXPECT_NEAR(factor.evaluateError(pose)(0), kOffset, kResidualTol);

  // Past a half turn the residual wraps into [-pi, pi] instead of growing
  constexpr double kWrappedOffset = 3.2;
  const gtsam::Rot3 wrapped_attitude =
      gtsam::Rot3::Yaw(magnetic_declination) * gtsam::Rot3::Yaw(sensor_yaw - kWrappedOffset);

  const AhrsYawFactorArm wrapped(pose_key, wrapped_attitude, target_T_sensor, magnetic_declination,
                                 model);

  EXPECT_NEAR(wrapped.evaluateError(pose)(0), kWrappedOffset - 2.0 * M_PI, kResidualTol);
}
