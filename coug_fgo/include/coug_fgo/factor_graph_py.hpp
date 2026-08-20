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

/**
 * @file factor_graph_py.hpp
 * @brief Thin Python bindings wrapper for the FactorGraphCore.
 * @author Nelson Durrant
 * @date May 2026
 */

#pragma once

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "coug_fgo/factor_graph_core.hpp"
#include "coug_fgo/factor_graph_parameters.hpp"

namespace coug_fgo {

/**
 * @class FactorGraphPy
 * @brief Thin Python bindings wrapper for the FactorGraphCore.
 */
class FactorGraphPy {
 public:
  using Vector6d = Eigen::Matrix<double, 6, 1>;
  using Matrix6d = Eigen::Matrix<double, 6, 6>;

  using ImuBatch = std::vector<
      std::tuple<double, Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d, Eigen::Matrix3d>>;
  using OdomBatch = std::vector<std::tuple<double, Eigen::Vector3d, Matrix6d>>;
  using DepthBatch = std::vector<std::tuple<double, double, Matrix6d>>;
  using MagBatch = std::vector<std::tuple<double, Eigen::Vector3d, Eigen::Matrix3d>>;
  using AhrsBatch = std::vector<std::tuple<double, Eigen::Vector4d, Eigen::Matrix3d>>;
  using TwistBatch = std::vector<std::tuple<double, Eigen::Vector3d, Matrix6d>>;
  using WrenchBatch = std::vector<std::tuple<double, Vector6d>>;
  using AgentStatus = std::tuple<double, Eigen::Vector3d, Eigen::Vector4d, Matrix6d, double,
                                 Eigen::Vector4d, bool, double, bool, double, double, bool, double>;
  using MultiAgentBatch = std::vector<std::vector<AgentStatus>>;
  using TfMap = std::unordered_map<std::string, std::pair<Eigen::Vector3d, Eigen::Vector4d>>;

  /**
   * @brief Constructs the wrapper, loading parameters from ROS 2 YAML config files.
   * @param config_paths Paths to ROS 2 parameter YAML files (later files override earlier ones).
   * @param ns Optional ROS namespace for parameter resolution.
   */
  explicit FactorGraphPy(const std::vector<std::string>& config_paths, const std::string& ns = "");

  /**
   * @brief Returns the loaded parameters needed to drive the offline pipeline.
   * @return Nested dict mirroring the node-level orchestration parameters.
   */
  pybind11::dict get_params() const;

  /**
   * @brief Seeds the graph from the newest sample in each measurement batch.
   * @param imu The IMU measurement batch.
   * @param gps The GPS measurement batch.
   * @param depth The depth measurement batch.
   * @param mag The magnetometer measurement batch.
   * @param ahrs The AHRS measurement batch.
   * @param dvl The DVL measurement batch.
   * @param wrench The wrench measurement batch.
   * @param multiagent The per-neighbor agent status batches.
   * @param tfs The resolved sensor transforms, keyed by sensor name.
   * @return True if the graph is initialized (or was already initialized).
   */
  bool initialize(const ImuBatch& imu, const OdomBatch& gps, const DepthBatch& depth,
                  const MagBatch& mag, const AhrsBatch& ahrs, const TwistBatch& dvl,
                  const WrenchBatch& wrench, const MultiAgentBatch& multiagent, const TfMap& tfs);

  /**
   * @brief Builds factors for one keyframe from the given measurement batches.
   * @param target_time The keyframe timestamp in seconds.
   * @param imu The IMU measurement batch.
   * @param gps The GPS measurement batch.
   * @param depth The depth measurement batch.
   * @param mag The magnetometer measurement batch.
   * @param ahrs The AHRS measurement batch.
   * @param dvl The DVL measurement batch.
   * @param wrench The wrench measurement batch.
   * @param multiagent The per-neighbor agent status batches.
   * @param tfs The resolved sensor transforms, keyed by sensor name.
   * @return Dict of batches to re-queue, or None if the keyframe was rejected (keep all queued).
   */
  pybind11::object update(double target_time, const ImuBatch& imu, const OdomBatch& gps,
                          const DepthBatch& depth, const MagBatch& mag, const AhrsBatch& ahrs,
                          const TwistBatch& dvl, const WrenchBatch& wrench,
                          const MultiAgentBatch& multiagent, const TfMap& tfs);

  /**
   * @brief Runs the GTSAM smoother on the buffered keyframes.
   * @return Optimization results, or an empty dict if there was nothing to optimize.
   */
  pybind11::dict optimize();

 private:
  // --- Core ---
  factor_graph_node::Params params_;
  std::unique_ptr<FactorGraphCore> core_;

  // --- State ---
  bool is_initialized_{false};
};

}  // namespace coug_fgo
