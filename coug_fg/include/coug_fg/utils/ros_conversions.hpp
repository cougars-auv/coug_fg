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

namespace coug_fg::utils {

inline gtsam::Matrix66 swapCovarianceBlocks(const gtsam::Matrix66& cov) {
  gtsam::Matrix66 swapped_cov;
  swapped_cov.topLeftCorner<3, 3>() = cov.bottomRightCorner<3, 3>();
  swapped_cov.bottomRightCorner<3, 3>() = cov.topLeftCorner<3, 3>();
  swapped_cov.topRightCorner<3, 3>() = cov.bottomLeftCorner<3, 3>();
  swapped_cov.bottomLeftCorner<3, 3>() = cov.topRightCorner<3, 3>();
  return swapped_cov;
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

inline gtsam::Matrix66 toGtsam(const std::array<double, 36>& cov) {
  return swapCovarianceBlocks(
      Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(cov.data()));
}

inline geometry_msgs::msg::Point toPointMsg(const gtsam::Point3& point) {
  geometry_msgs::msg::Point point_msg;
  point_msg.x = point.x();
  point_msg.y = point.y();
  point_msg.z = point.z();
  return point_msg;
}

inline geometry_msgs::msg::Vector3 toVectorMsg(const gtsam::Vector3& vector) {
  geometry_msgs::msg::Vector3 vector_msg;
  vector_msg.x = vector.x();
  vector_msg.y = vector.y();
  vector_msg.z = vector.z();
  return vector_msg;
}

inline geometry_msgs::msg::Quaternion toQuatMsg(const gtsam::Rot3& rotation) {
  gtsam::Quaternion quat = rotation.toQuaternion();
  geometry_msgs::msg::Quaternion quat_msg;
  quat_msg.w = quat.w();
  quat_msg.x = quat.x();
  quat_msg.y = quat.y();
  quat_msg.z = quat.z();
  return quat_msg;
}

inline geometry_msgs::msg::Pose toPoseMsg(const gtsam::Pose3& pose) {
  geometry_msgs::msg::Pose pose_msg;
  pose_msg.position = toPointMsg(pose.translation());
  pose_msg.orientation = toQuatMsg(pose.rotation());
  return pose_msg;
}

inline std::array<double, 36> toCovariance36Msg(const gtsam::Matrix33& cov) {
  std::array<double, 36> cov_msg{};
  cov_msg.fill(0.0);
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      cov_msg[i * 6 + j] = cov(i, j);
    }
  }
  return cov_msg;
}

inline std::array<double, 9> toCovariance9Msg(const gtsam::Matrix33& cov) {
  std::array<double, 9> cov_msg{};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      cov_msg[i * 3 + j] = cov(i, j);
    }
  }
  return cov_msg;
}

inline std::array<double, 36> toCovariance36Msg(const gtsam::Matrix66& cov) {
  std::array<double, 36> cov_msg{};
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      cov_msg[i * 6 + j] = cov(i, j);
    }
  }
  return cov_msg;
}

inline std::array<double, 36> toPoseCovarianceMsg(const gtsam::Matrix66& cov) {
  return toCovariance36Msg(swapCovarianceBlocks(cov));
}

}  // namespace coug_fg::utils
