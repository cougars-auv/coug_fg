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

#include "coug_fg/factors/wrench_dynamics_factor.hpp"

namespace {

using coug_fg::factors::WrenchDynamicsFactorArm;

using gtsam::symbol_shorthand::V;  // Velocity (x,y,z)
using gtsam::symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)

constexpr double kStep = 1e-5;  // finite difference step
constexpr double kJacobianTol = 1e-5;
constexpr double kResidualTol = 1e-9;

}  // namespace

TEST(WrenchDynamicsFactorArmTest, Jacobians) {
  gtsam::Key pose_key_i = X(1);
  gtsam::Key vel_key_i = V(1);
  gtsam::Key pose_key_j = X(2);
  gtsam::Key vel_key_j = V(2);
  gtsam::SharedNoiseModel model = gtsam::noiseModel::Isotropic::Sigma(3, 0.1);

  double dt = 0.5;
  gtsam::Matrix33 mass = gtsam::Matrix33::Identity() * 5.0;
  gtsam::Matrix33 linear_drag = gtsam::Matrix33::Identity() * 1.0;
  gtsam::Matrix33 quad_drag = gtsam::Matrix33::Identity() * 0.5;
  gtsam::Vector3 control_force(2.0, -1.0, 0.5);
  gtsam::Pose3 target_T_sensor(gtsam::Rot3::Ypr(0.1, -0.1, 0.1), gtsam::Point3(0.5, 0.5, 0.5));

  WrenchDynamicsFactorArm factor(pose_key_i, vel_key_i, pose_key_j, vel_key_j, dt, control_force,
                                 target_T_sensor, mass, linear_drag, quad_drag, model);

  gtsam::Values values;
  values.insert(pose_key_i,
                gtsam::Pose3(gtsam::Rot3::Ypr(0.1, 0.2, 0.3), gtsam::Point3(1.0, 2.0, 4.0)));
  values.insert(vel_key_i, gtsam::Vector3(1.0, -0.5, 0.2));
  values.insert(pose_key_j,
                gtsam::Pose3(gtsam::Rot3::Ypr(0.4, -0.1, 0.2), gtsam::Point3(2.0, 3.0, 4.0)));
  values.insert(vel_key_j, gtsam::Vector3(1.1, -0.4, 0.25));

  EXPECT_TRUE(gtsam::internal::testFactorJacobians("WrenchDynamicsFactorArm", factor, values, kStep,
                                                   kJacobianTol));
}

TEST(WrenchDynamicsFactorArmTest, Residual) {
  gtsam::Key pose_key_i = X(1);
  gtsam::Key vel_key_i = V(1);
  gtsam::Key pose_key_j = X(2);
  gtsam::Key vel_key_j = V(2);
  gtsam::SharedNoiseModel model = gtsam::noiseModel::Isotropic::Sigma(3, 0.1);

  double dt = 0.5;
  gtsam::Matrix33 mass = gtsam::Matrix33::Identity() * 5.0;
  gtsam::Matrix33 linear_drag = gtsam::Matrix33::Identity() * 1.0;
  gtsam::Matrix33 quad_drag = gtsam::Matrix33::Identity() * 0.5;
  gtsam::Vector3 control_force(2.0, -1.0, 0.5);
  gtsam::Pose3 target_T_sensor(gtsam::Rot3::Ypr(0.1, -0.1, 0.1), gtsam::Point3(0.5, 0.5, 0.5));

  WrenchDynamicsFactorArm factor(pose_key_i, vel_key_i, pose_key_j, vel_key_j, dt, control_force,
                                 target_T_sensor, mass, linear_drag, quad_drag, model);

  gtsam::Pose3 pose_i(gtsam::Rot3::Ypr(0.1, 0.2, 0.3), gtsam::Point3(1.0, 2.0, 4.0));
  gtsam::Pose3 pose_j(gtsam::Rot3::Ypr(0.4, -0.1, 0.2), gtsam::Point3(2.0, 3.0, 4.0));
  gtsam::Vector3 vel_i(1.0, -0.5, 0.2);

  // Step the Fossen model forward once
  const gtsam::Vector3 target_force = target_T_sensor.rotation().matrix() * control_force;
  const gtsam::Vector3 vel_target_i = pose_i.rotation().matrix().transpose() * vel_i;
  const gtsam::Vector3 drag_force = -(
      linear_drag * vel_target_i + quad_drag * vel_target_i.cwiseProduct(vel_target_i.cwiseAbs()));
  const gtsam::Vector3 vel_target_pred =
      vel_target_i + mass.inverse() * (target_force + drag_force) * dt;

  // Pick vel_j so the residual is the offset
  const gtsam::Vector3 offset(0.02, -0.01, 0.03);
  const gtsam::Vector3 vel_j = pose_j.rotation().matrix() * (vel_target_pred + offset);

  const gtsam::Vector expected = offset;
  EXPECT_TRUE(gtsam::assert_equal(expected, factor.evaluateError(pose_i, vel_i, pose_j, vel_j),
                                  kResidualTol));
}
