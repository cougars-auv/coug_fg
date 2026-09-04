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
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/factorTesting.h>

#include <cmath>

#include "coug_fg/factors/bearing_factor.hpp"

namespace {

using coug_fg::factors::BearingFactorArm;

using gtsam::symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)

constexpr double kStep = 1e-5;  // finite difference step
constexpr double kJacobianTol = 1e-5;
constexpr double kResidualTol = 1e-9;

}  // namespace

TEST(BearingFactorArmTest, Jacobians) {
  gtsam::Key const pose_key_l = X(1);
  gtsam::Key const pose_key_n = X(2);
  gtsam::SharedNoiseModel const model = gtsam::noiseModel::Isotropic::Sigma(2, 0.1);
  gtsam::Pose3 const target_T_sensor_l(gtsam::Rot3::Ypr(0.1, -0.1, 0.1),
                                       gtsam::Point3(0.5, 0.5, 0.5));
  gtsam::Pose3 const target_T_sensor_n(gtsam::Rot3::Ypr(-0.2, 0.1, 0.3),
                                       gtsam::Point3(0.2, -0.4, 0.1));
  gtsam::Point2 const measured_azi_el(0.3, 0.2);

  BearingFactorArm const factor(pose_key_l, pose_key_n, measured_azi_el, target_T_sensor_l,
                                target_T_sensor_n, model);

  gtsam::Values values;
  values.insert(pose_key_l,
                gtsam::Pose3(gtsam::Rot3::Ypr(0.1, 0.2, 0.3), gtsam::Point3(1.0, 2.0, 4.0)));
  values.insert(pose_key_n,
                gtsam::Pose3(gtsam::Rot3::Ypr(0.4, -0.1, 0.2), gtsam::Point3(8.0, 5.0, 6.0)));

  EXPECT_TRUE(gtsam::internal::testFactorJacobians("BearingFactorArm", factor, values, kStep,
                                                   kJacobianTol));
}

TEST(BearingFactorArmTest, Residual) {
  gtsam::Key const pose_key_l = X(1);
  gtsam::Key const pose_key_n = X(2);
  gtsam::SharedNoiseModel const model = gtsam::noiseModel::Isotropic::Sigma(2, 0.1);
  gtsam::Pose3 const target_T_sensor_l(gtsam::Rot3::Ypr(0.1, -0.1, 0.1),
                                       gtsam::Point3(0.5, 0.5, 0.5));
  gtsam::Pose3 const target_T_sensor_n(gtsam::Rot3::Ypr(-0.2, 0.1, 0.3),
                                       gtsam::Point3(0.2, -0.4, 0.1));
  gtsam::Pose3 const pose_l(gtsam::Rot3::Ypr(0.1, 0.2, 0.3), gtsam::Point3(1.0, 2.0, 4.0));
  gtsam::Pose3 const pose_n(gtsam::Rot3::Ypr(0.4, -0.1, 0.2), gtsam::Point3(8.0, 5.0, 6.0));

  // Angles the local sensor sees the neighbor's sensor at, inverting the line of sight
  const gtsam::Pose3 map_T_sensor_l = pose_l * target_T_sensor_l;
  const gtsam::Pose3 map_T_sensor_n = pose_n * target_T_sensor_n;
  const gtsam::Vector3 los = map_T_sensor_l.rotation().matrix().transpose() *
                             (map_T_sensor_n.translation() - map_T_sensor_l.translation());
  const double azimuth = std::atan2(los.y(), los.x());
  const double elevation = std::asin(los.z() / los.norm());

  // Matching the state exactly leaves no residual
  BearingFactorArm const factor(pose_key_l, pose_key_n, gtsam::Point2(azimuth, elevation),
                                target_T_sensor_l, target_T_sensor_n, model);

  const gtsam::Vector expected = gtsam::Vector2::Zero();
  EXPECT_TRUE(gtsam::assert_equal(expected, factor.evaluateError(pose_l, pose_n), kResidualTol));

  // Elevation is an arc length, so tilting by a known angle moves the residual by that angle
  constexpr double kOffset = 0.01;  // [rad]
  BearingFactorArm const tilted(pose_key_l, pose_key_n, gtsam::Point2(azimuth, elevation - kOffset),
                                target_T_sensor_l, target_T_sensor_n, model);

  EXPECT_NEAR(gtsam::Vector(tilted.evaluateError(pose_l, pose_n)).norm(), kOffset, 1e-6);
}
