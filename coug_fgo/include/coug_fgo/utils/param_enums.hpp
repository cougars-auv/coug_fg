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

#include <stdexcept>
#include <string>

namespace coug_fgo::utils {

enum class SolverType { kIncrementalFixedLagSmoother, kIsam2, kLevenbergMarquardt };

enum class RobustKernel { kNone, kHuber, kTukey };

enum class KeyframeSource { kNone, kDvl, kDepth, kTimer };

inline SolverType parseSolverType(const std::string& solver_type) {
  if (solver_type == "IncrementalFixedLagSmoother") return SolverType::kIncrementalFixedLagSmoother;
  if (solver_type == "ISAM2") return SolverType::kIsam2;
  if (solver_type == "LevenbergMarquardt") return SolverType::kLevenbergMarquardt;
  throw std::invalid_argument("Unknown solver_type: " + solver_type);
}

inline RobustKernel parseRobustKernel(const std::string& robust_kernel) {
  if (robust_kernel == "None") return RobustKernel::kNone;
  if (robust_kernel == "Huber") return RobustKernel::kHuber;
  if (robust_kernel == "Tukey") return RobustKernel::kTukey;
  throw std::invalid_argument("Unknown robust_kernel: " + robust_kernel);
}

inline KeyframeSource parseKeyframeSource(const std::string& keyframe_source) {
  if (keyframe_source == "None") return KeyframeSource::kNone;
  if (keyframe_source == "DVL") return KeyframeSource::kDvl;
  if (keyframe_source == "Depth") return KeyframeSource::kDepth;
  if (keyframe_source == "Timer") return KeyframeSource::kTimer;
  throw std::invalid_argument("Unknown keyframe source: " + keyframe_source);
}

}  // namespace coug_fgo::utils
