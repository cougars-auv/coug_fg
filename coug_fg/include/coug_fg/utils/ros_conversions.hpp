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

inline auto swapCovarianceBlocks(gtsam::Matrix66 const& cov) -> gtsam::Matrix66 {
  gtsam::Matrix66 swapped_cov;
  swapped_cov.topLeftCorner<3, 3>() = cov.bottomRightCorner<3, 3>();
  swapped_cov.bottomRightCorner<3, 3>() = cov.topLeftCorner<3, 3>();
  swapped_cov.topRightCorner<3, 3>() = cov.bottomLeftCorner<3, 3>();
  swapped_cov.bottomLeftCorner<3, 3>() = cov.topRightCorner<3, 3>();
  return swapped_cov;
}

inline auto toGtsam(geometry_msgs::msg::Point const& msg) -> gtsam::Point3 {
  return {msg.x, msg.y, msg.z};
}

inline auto toGtsam(geometry_msgs::msg::Vector3 const& msg) -> gtsam::Vector3 {
  return {msg.x, msg.y, msg.z};
}

inline auto toGtsam(geometry_msgs::msg::Quaternion const& msg) -> gtsam::Rot3 {
  return gtsam::Rot3::Quaternion(msg.w, msg.x, msg.y, msg.z);
}

inline auto toGtsam(geometry_msgs::msg::Pose const& msg) -> gtsam::Pose3 {
  return {toGtsam(msg.orientation), toGtsam(msg.position)};
}

inline auto toGtsam(geometry_msgs::msg::Transform const& msg) -> gtsam::Pose3 {
  return {toGtsam(msg.rotation), toGtsam(msg.translation)};
}

inline auto toGtsam(std::array<double, 36> const& cov) -> gtsam::Matrix66 {
  return swapCovarianceBlocks(
      Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor> const>(cov.data()));
}

inline auto toPointMsg(gtsam::Point3 const& point) -> geometry_msgs::msg::Point {
  geometry_msgs::msg::Point point_msg;
  point_msg.x = point.x();
  point_msg.y = point.y();
  point_msg.z = point.z();
  return point_msg;
}

inline auto toVectorMsg(gtsam::Vector3 const& vector) -> geometry_msgs::msg::Vector3 {
  geometry_msgs::msg::Vector3 vector_msg;
  vector_msg.x = vector.x();
  vector_msg.y = vector.y();
  vector_msg.z = vector.z();
  return vector_msg;
}

inline auto toQuatMsg(gtsam::Rot3 const& rotation) -> geometry_msgs::msg::Quaternion {
  gtsam::Quaternion quat = rotation.toQuaternion();
  geometry_msgs::msg::Quaternion quat_msg;
  quat_msg.w = quat.w();
  quat_msg.x = quat.x();
  quat_msg.y = quat.y();
  quat_msg.z = quat.z();
  return quat_msg;
}

inline auto toPoseMsg(gtsam::Pose3 const& pose) -> geometry_msgs::msg::Pose {
  geometry_msgs::msg::Pose pose_msg;
  pose_msg.position = toPointMsg(pose.translation());
  pose_msg.orientation = toQuatMsg(pose.rotation());
  return pose_msg;
}

inline auto toCovariance36Msg(gtsam::Matrix33 const& cov) -> std::array<double, 36> {
  std::array<double, 36> cov_msg{};
  cov_msg.fill(0.0);
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      cov_msg[i * 6 + j] = cov(i, j);
    }
  }
  return cov_msg;
}

inline auto toCovariance9Msg(gtsam::Matrix33 const& cov) -> std::array<double, 9> {
  std::array<double, 9> cov_msg{};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      cov_msg[i * 3 + j] = cov(i, j);
    }
  }
  return cov_msg;
}

inline auto toCovariance36Msg(gtsam::Matrix66 const& cov) -> std::array<double, 36> {
  std::array<double, 36> cov_msg{};
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      cov_msg[i * 6 + j] = cov(i, j);
    }
  }
  return cov_msg;
}

inline auto toPoseCovarianceMsg(gtsam::Matrix66 const& cov) -> std::array<double, 36> {
  return toCovariance36Msg(swapCovarianceBlocks(cov));
}

}  // namespace coug_fg::utils
