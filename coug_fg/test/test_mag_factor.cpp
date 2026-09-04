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
#include <gtsam/base/Vector.h>
#include <gtsam/base/types.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/factorTesting.h>

#include "coug_fg/factors/mag_factor.hpp"

namespace {

using coug_fg::factors::MagFactorArm;

using gtsam::symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)

constexpr double kStep = 1e-5;          // finite difference step
constexpr double kJacobianTol = 1e-10;  // fields run ~1e-5 T, a loose tol accepts zero
constexpr double kResidualTol = 1e-15;

}  // namespace

TEST(MagFactorArmTest, Jacobians) {
  gtsam::Key const pose_key = X(1);
  gtsam::SharedNoiseModel const model = gtsam::noiseModel::Isotropic::Sigma(3, 0.1);
  gtsam::Pose3 const target_T_sensor(gtsam::Rot3::Ypr(0.1, -0.1, 0.1), gtsam::Point3::Zero());
  gtsam::Point3 const reference_field(3.9634e-06, 2.08423e-05, -4.57678e-05);
  gtsam::Point3 const measured_field(4.1000e-06, 2.00000e-05, -4.50000e-05);

  MagFactorArm const factor(pose_key, measured_field, reference_field, target_T_sensor, model);

  gtsam::Values values;
  values.insert(pose_key,
                gtsam::Pose3(gtsam::Rot3::Ypr(0.1, 0.2, 0.3), gtsam::Point3(1.0, 2.0, 4.0)));

  EXPECT_TRUE(
      gtsam::internal::testFactorJacobians("MagFactorArm", factor, values, kStep, kJacobianTol));
}

TEST(MagFactorArmTest, Residual) {
  gtsam::Key const pose_key = X(1);
  gtsam::SharedNoiseModel const model = gtsam::noiseModel::Isotropic::Sigma(3, 0.1);
  gtsam::Pose3 const target_T_sensor(gtsam::Rot3::Ypr(0.1, -0.1, 0.1), gtsam::Point3::Zero());
  gtsam::Pose3 const pose(gtsam::Rot3::Ypr(0.1, 0.2, 0.3), gtsam::Point3(1.0, 2.0, 4.0));
  gtsam::Point3 const reference_field(3.9634e-06, 2.08423e-05, -4.57678e-05);

  // Field the sensor would report if the state were exact
  const gtsam::Rot3 map_R_sensor = pose.rotation() * target_T_sensor.rotation();
  const gtsam::Point3 sensor_field = map_R_sensor.matrix().transpose() * reference_field;

  // Measured short of the prediction
  const gtsam::Vector3 offset(1.0e-7, -2.0e-7, 3.0e-7);
  MagFactorArm const factor(pose_key, sensor_field - offset, reference_field, target_T_sensor,
                            model);

  const gtsam::Vector expected = offset;
  EXPECT_TRUE(gtsam::assert_equal(expected, factor.evaluateError(pose), kResidualTol));
}
