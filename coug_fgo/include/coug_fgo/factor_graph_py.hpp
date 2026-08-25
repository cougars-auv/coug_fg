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

class FactorGraphPy {
 public:
  using Vector6d = Eigen::Matrix<double, 6, 1>;
  using Matrix6d = Eigen::Matrix<double, 6, 6>;

  using ImuBatch = std::vector<
      std::tuple<double, Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d, Eigen::Matrix3d>>;
  using GpsBatch = std::vector<std::tuple<double, Eigen::Vector3d, Matrix6d>>;
  using DepthBatch = std::vector<std::tuple<double, double, Matrix6d>>;
  using MagBatch = std::vector<std::tuple<double, Eigen::Vector3d, Eigen::Matrix3d>>;
  using AhrsBatch = std::vector<std::tuple<double, Eigen::Vector4d, Eigen::Matrix3d>>;
  using DvlBatch = std::vector<std::tuple<double, Eigen::Vector3d, Matrix6d>>;
  using WrenchBatch = std::vector<std::tuple<double, Vector6d>>;
  using AgentStatus = std::tuple<double, Eigen::Vector3d, Eigen::Vector4d, Matrix6d, double,
                                 Eigen::Vector4d, bool, double, bool, double, double, bool, double>;
  using MultiAgentBatch = std::vector<std::vector<AgentStatus>>;
  using TfMap = std::unordered_map<std::string, std::pair<Eigen::Vector3d, Eigen::Vector4d>>;

  explicit FactorGraphPy(const std::vector<std::string>& config_paths, const std::string& ns = "");

  pybind11::dict get_params() const;

  bool initialize(double init_time, const ImuBatch& imu, const GpsBatch& gps,
                  const DepthBatch& depth, const MagBatch& mag, const AhrsBatch& ahrs,
                  const DvlBatch& dvl, const WrenchBatch& wrench, const MultiAgentBatch& multiagent,
                  const TfMap& tfs);

  pybind11::object update(double target_time, const ImuBatch& imu, const GpsBatch& gps,
                          const DepthBatch& depth, const MagBatch& mag, const AhrsBatch& ahrs,
                          const DvlBatch& dvl, const WrenchBatch& wrench,
                          const MultiAgentBatch& multiagent, const TfMap& tfs);

  pybind11::dict optimize();

 private:
  // --- Core ---
  factor_graph_node::Params params_;
  std::unique_ptr<FactorGraphCore> core_;

  // --- State ---
  bool is_initialized_{false};
};

}  // namespace coug_fgo
