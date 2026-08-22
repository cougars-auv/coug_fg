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

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>

#include <array>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/vector3.hpp>

namespace coug_fgo::utils {

inline gtsam::Matrix66 swapCovarianceBlocks(const gtsam::Matrix66& cov) {
  gtsam::Matrix66 swapped;
  swapped.topLeftCorner<3, 3>() = cov.bottomRightCorner<3, 3>();
  swapped.bottomRightCorner<3, 3>() = cov.topLeftCorner<3, 3>();
  swapped.topRightCorner<3, 3>() = cov.bottomLeftCorner<3, 3>();
  swapped.bottomLeftCorner<3, 3>() = cov.topRightCorner<3, 3>();
  return swapped;
}

inline gtsam::Point3 toGtsam(const geometry_msgs::msg::Point& msg) { return {msg.x, msg.y, msg.z}; }

inline gtsam::Vector3 toGtsam(const geometry_msgs::msg::Vector3& msg) {
  return {msg.x, msg.y, msg.z};
}

inline gtsam::Rot3 toGtsam(const geometry_msgs::msg::Quaternion& msg) {
  return gtsam::Rot3::Quaternion(msg.w, msg.x, msg.y, msg.z);
}

inline gtsam::Pose3 toGtsam(const geometry_msgs::msg::Pose& msg) {
  return {toGtsam(msg.orientation), toGtsam(msg.position)};
}

inline gtsam::Pose3 toGtsam(const geometry_msgs::msg::Transform& msg) {
  return {toGtsam(msg.rotation), toGtsam(msg.translation)};
}

inline gtsam::Matrix66 toGtsam(const std::array<double, 36>& msg) {
  return swapCovarianceBlocks(
      Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(msg.data()));
}

inline geometry_msgs::msg::Point toPointMsg(const gtsam::Point3& gtsam_obj) {
  geometry_msgs::msg::Point msg;
  msg.x = gtsam_obj.x();
  msg.y = gtsam_obj.y();
  msg.z = gtsam_obj.z();
  return msg;
}

inline geometry_msgs::msg::Vector3 toVectorMsg(const gtsam::Vector3& gtsam_obj) {
  geometry_msgs::msg::Vector3 msg;
  msg.x = gtsam_obj.x();
  msg.y = gtsam_obj.y();
  msg.z = gtsam_obj.z();
  return msg;
}

inline geometry_msgs::msg::Quaternion toQuatMsg(const gtsam::Rot3& gtsam_obj) {
  gtsam::Quaternion q = gtsam_obj.toQuaternion();
  geometry_msgs::msg::Quaternion msg;
  msg.w = q.w();
  msg.x = q.x();
  msg.y = q.y();
  msg.z = q.z();
  return msg;
}

inline geometry_msgs::msg::Pose toPoseMsg(const gtsam::Pose3& gtsam_obj) {
  geometry_msgs::msg::Pose msg;
  msg.position = toPointMsg(gtsam_obj.translation());
  msg.orientation = toQuatMsg(gtsam_obj.rotation());
  return msg;
}

inline std::array<double, 36> toCovariance36Msg(const gtsam::Matrix33& cov) {
  std::array<double, 36> msg;
  msg.fill(0.0);
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      msg[i * 6 + j] = cov(i, j);
    }
  }
  return msg;
}

inline std::array<double, 9> toCovariance9Msg(const gtsam::Matrix33& cov) {
  std::array<double, 9> msg;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      msg[i * 3 + j] = cov(i, j);
    }
  }
  return msg;
}

inline std::array<double, 36> toCovariance36Msg(const gtsam::Matrix66& cov) {
  std::array<double, 36> msg;
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      msg[i * 6 + j] = cov(i, j);
    }
  }
  return msg;
}

inline std::array<double, 36> toPoseCovarianceMsg(const gtsam::Matrix66& cov) {
  return toCovariance36Msg(swapCovarianceBlocks(cov));
}

}  // namespace coug_fgo::utils
