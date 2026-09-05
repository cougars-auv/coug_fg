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
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/factorTesting.h>

#include "coug_fg/factors/const_vel_factor.hpp"

namespace {

using coug_fg::factors::ConstVelFactor;

using gtsam::symbol_shorthand::V;  // Velocity (x,y,z)
using gtsam::symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)

constexpr double kStep = 1e-5;  // finite difference step
constexpr double kJacobianTol = 1e-5;
constexpr double kResidualTol = 1e-9;

}  // namespace

TEST(ConstVelFactorTest, Jacobians) {
  const gtsam::Key pose_key_i = X(1);
  const gtsam::Key vel_key_i = V(1);
  const gtsam::Key pose_key_j = X(2);
  const gtsam::Key vel_key_j = V(2);
  const gtsam::SharedNoiseModel model = gtsam::noiseModel::Isotropic::Sigma(3, 0.1);

  const ConstVelFactor factor(pose_key_i, vel_key_i, pose_key_j, vel_key_j, model);

  gtsam::Values values;
  values.insert(pose_key_i,
                gtsam::Pose3(gtsam::Rot3::Ypr(0.1, 0.2, 0.3), gtsam::Point3(1.0, 2.0, 4.0)));
  values.insert(vel_key_i, gtsam::Vector3(1.0, 0.5, 0.0));
  values.insert(pose_key_j,
                gtsam::Pose3(gtsam::Rot3::Ypr(0.4, -0.1, 0.2), gtsam::Point3(2.0, 3.0, 4.0)));
  values.insert(vel_key_j, gtsam::Vector3(1.1, 0.4, 0.1));

  EXPECT_TRUE(
      gtsam::internal::testFactorJacobians("ConstVelFactor", factor, values, kStep, kJacobianTol));
}

TEST(ConstVelFactorTest, Residual) {
  const gtsam::Key pose_key_i = X(1);
  const gtsam::Key vel_key_i = V(1);
  const gtsam::Key pose_key_j = X(2);
  const gtsam::Key vel_key_j = V(2);
  const gtsam::SharedNoiseModel model = gtsam::noiseModel::Isotropic::Sigma(3, 0.1);

  const ConstVelFactor factor(pose_key_i, vel_key_i, pose_key_j, vel_key_j, model);

  const gtsam::Pose3 pose_i(gtsam::Rot3::Ypr(0.1, 0.2, 0.3), gtsam::Point3(1.0, 2.0, 4.0));
  const gtsam::Pose3 pose_j(gtsam::Rot3::Ypr(0.4, -0.1, 0.2), gtsam::Point3(2.0, 3.0, 4.0));
  const gtsam::Vector3 vel_i(1.0, 0.5, 0.0);

  // Compared in each pose's own target frame, not the map frame
  const gtsam::Vector3 vel_target_i = pose_i.rotation().matrix().transpose() * vel_i;

  // Pick vel_j so the target-frame difference is the offset
  const gtsam::Vector3 offset(0.02, -0.01, 0.03);
  const gtsam::Vector3 vel_j = pose_j.rotation().matrix() * (vel_target_i - offset);

  const gtsam::Vector expected = offset;
  EXPECT_TRUE(gtsam::assert_equal(expected, factor.evaluateError(pose_i, vel_i, pose_j, vel_j),
                                  kResidualTol));
}
