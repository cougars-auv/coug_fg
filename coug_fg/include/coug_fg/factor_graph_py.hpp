// Copyright 2026 BYU FROST Lab
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

#include <memory>
#include <string>
#include <vector>

#include "coug_fg/factor_graph_core.hpp"
#include "coug_fg/factor_graph_parameters.hpp"

namespace coug_fg {

class FactorGraphPy {
 public:
  explicit FactorGraphPy(const std::vector<std::string>& config_paths, const std::string& ns = "");

  [[nodiscard]] auto get_params() const -> pybind11::dict;

  auto initialize(double init_time, const pybind11::dict& queues, const pybind11::dict& tfs)
      -> bool;

  auto update(double target_time, const pybind11::dict& queues, const pybind11::dict& tfs)
      -> pybind11::object;

  auto optimize() -> pybind11::dict;

 private:
  // --- Core ---
  factor_graph_node::Params params_;
  std::unique_ptr<FactorGraphCore> core_;

  // --- State ---
  bool is_initialized_{false};
};

}  // namespace coug_fg
