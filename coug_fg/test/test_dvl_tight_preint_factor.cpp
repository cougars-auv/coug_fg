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
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/types.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/factorTesting.h>

#include "coug_fg/factors/dvl_tight_preint_factor.hpp"

namespace {

using coug_fg::factors::DvlTightPreintFactorArm;

using gtsam::symbol_shorthand::B;  // Bias (ax,ay,az,gx,gy,gz)
using gtsam::symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)

constexpr double kStep = 1e-5;  // finite difference step
constexpr double kJacobianTol = 1e-5;
constexpr double kResidualTol = 1e-9;

}  // namespace

TEST(DvlTightPreintFactorArmTest, Jacobians) {
  gtsam::Key const pose_key_i = X(1);
  gtsam::Key const pose_key_j = X(2);
  gtsam::Key const bias_key_i = B(1);
  gtsam::SharedNoiseModel const model = gtsam::noiseModel::Isotropic::Sigma(3, 0.1);
  gtsam::Pose3 const target_T_dvl(gtsam::Rot3::Ypr(-0.1, 0.1, -0.1), gtsam::Point3(0.5, 0.5, 0.5));
  gtsam::Vector3 const measured_translation(1.0, 0.5, -0.2);
  gtsam::Matrix3 const J_p_bg = gtsam::Matrix3::Identity() * 0.01;
  gtsam::Vector3 const gyro_bias_hat(0.01, -0.02, 0.005);

  DvlTightPreintFactorArm const factor(pose_key_i, pose_key_j, bias_key_i, target_T_dvl,
                                       measured_translation, J_p_bg, gyro_bias_hat, model);

  gtsam::Values values;
  values.insert(pose_key_i,
                gtsam::Pose3(gtsam::Rot3::Ypr(0.1, 0.2, 0.3), gtsam::Point3(1.0, 2.0, 4.0)));
  values.insert(pose_key_j,
                gtsam::Pose3(gtsam::Rot3::Ypr(-0.2, 0.4, 0.1), gtsam::Point3(2.0, 3.0, 2.5)));
  values.insert(bias_key_i, gtsam::imuBias::ConstantBias(gtsam::Vector3(0.0, 0.0, 0.0),
                                                         gtsam::Vector3(0.02, -0.01, 0.01)));

  EXPECT_TRUE(gtsam::internal::testFactorJacobians("DvlTightPreintFactorArm", factor, values, kStep,
                                                   kJacobianTol));
}

TEST(DvlTightPreintFactorArmTest, Residual) {
  gtsam::Key const pose_key_i = X(1);
  gtsam::Key const pose_key_j = X(2);
  gtsam::Key const bias_key_i = B(1);
  gtsam::SharedNoiseModel const model = gtsam::noiseModel::Isotropic::Sigma(3, 0.1);
  gtsam::Pose3 const target_T_dvl(gtsam::Rot3::Ypr(-0.1, 0.1, -0.1), gtsam::Point3(0.5, 0.5, 0.5));
  gtsam::Matrix3 const J_p_bg = gtsam::Matrix3::Identity() * 0.01;
  gtsam::Vector3 const gyro_bias_hat(0.01, -0.02, 0.005);

  gtsam::Pose3 const pose_i(gtsam::Rot3::Ypr(0.1, 0.2, 0.3), gtsam::Point3(1.0, 2.0, 4.0));
  gtsam::Pose3 const pose_j(gtsam::Rot3::Ypr(-0.2, 0.4, 0.1), gtsam::Point3(2.0, 3.0, 2.5));
  gtsam::imuBias::ConstantBias const bias_i(gtsam::Vector3(0.0, 0.0, 0.0),
                                            gtsam::Vector3(0.02, -0.01, 0.01));

  // DVL travel between the poses, in the target frame at i
  gtsam::Point3 const dvl_map_j =
      pose_j.rotation().matrix() * target_T_dvl.translation() + pose_j.translation();
  gtsam::Vector3 const i_p_sensor_j =
      pose_i.rotation().matrix().transpose() * (dvl_map_j - pose_i.translation());
  gtsam::Vector3 const predicted_translation = i_p_sensor_j - target_T_dvl.translation();

  // The factor re-corrects for bias drift since preintegration
  gtsam::Vector3 const bias_correction = J_p_bg * (bias_i.gyroscope() - gyro_bias_hat);

  // Measured short of the corrected prediction
  gtsam::Vector3 const offset(0.01, -0.02, 0.03);
  DvlTightPreintFactorArm const factor(pose_key_i, pose_key_j, bias_key_i, target_T_dvl,
                                       predicted_translation - bias_correction - offset, J_p_bg,
                                       gyro_bias_hat, model);

  gtsam::Vector const expected = offset;
  EXPECT_TRUE(
      gtsam::assert_equal(expected, factor.evaluateError(pose_i, pose_j, bias_i), kResidualTol));
}
