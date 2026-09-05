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

#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "coug_fg/factor_graph_parameters.hpp"
#include "coug_fg/utils/data_types.hpp"
#include "coug_fg/utils/dvl_loose_preintegrator.hpp"
#include "coug_fg/utils/dvl_tight_preintegrator.hpp"
#include "coug_fg/utils/logger.hpp"

namespace coug_fg {

struct NeighborResult {
  size_t agent_queue_idx{};
  double timestamp{0.0};
  gtsam::Key pose_key{0};

  gtsam::Pose3 pose;
  gtsam::Matrix pose_cov;
  std::optional<gtsam::Pose3> origin_delta;
};

struct OptimizeResult {
  double timestamp{0.0};
  gtsam::Pose3 pose;
  gtsam::Vector3 velocity;
  gtsam::imuBias::ConstantBias imu_bias;
  gtsam::Point3 mag_bias;
  gtsam::Matrix pose_cov;
  gtsam::Matrix velocity_cov;
  gtsam::Matrix imu_bias_cov;
  gtsam::Matrix mag_bias_cov;
  gtsam::Values smoothed_path;

  double total_duration{0.0};
  double smoother_duration{0.0};
  double cov_duration{0.0};
  bool processing_overflow{false};

  size_t new_keyframes{0};
  size_t new_factors{0};
  size_t total_factors{0};
  size_t total_variables{0};

  std::vector<NeighborResult> neighbor_results;
};

class FactorGraphCore {
 public:
  explicit FactorGraphCore(factor_graph_node::Params params);

  void setLogCallback(utils::LogCallback callback);

  auto initialize(double init_time, utils::QueueBundle const& queues, utils::TfBundle const& tfs)
      -> bool;

  auto update(double target_time, utils::QueueBundle& queues, utils::TfBundle const& tfs)
      -> std::optional<utils::QueueBundle>;

  auto optimize() -> std::optional<OptimizeResult>;

  auto snapshotTimeKeys() const -> std::map<int64_t, gtsam::Key>;

 private:
  struct InitialState {
    gtsam::Pose3 pose;
    gtsam::Vector3 velocity;
    gtsam::imuBias::ConstantBias imu_bias;
    gtsam::Point3 mag_bias;
    double timestamp{0.0};

    gtsam::Matrix6 pose_cov;
    gtsam::Matrix3 velocity_cov;
    gtsam::Matrix6 imu_bias_cov;
    gtsam::Matrix3 mag_bias_cov;

    std::shared_ptr<utils::ImuData> imu;
    std::shared_ptr<utils::TwistData> dvl;
  };

  struct NeighborState {
    static constexpr size_t kStepShift = 32;

    explicit NeighborState(size_t agent_queue_idx) : curr_step(agent_queue_idx << kStepShift) {}

    void initialize(gtsam::Pose3 const& pose, gtsam::Matrix66 const& cov, double time) {
      prev_pose = pose;
      prev_cov = cov;

      curr_pose = pose;
      curr_cov = cov;
      curr_time = time;
    }

    void advance(gtsam::Pose3 const& new_pose, gtsam::Matrix66 const& new_cov, double new_time) {
      ++curr_step;

      prev_pose = curr_pose;
      prev_cov = curr_cov;

      curr_pose = new_pose;
      curr_cov = new_cov;
      curr_time = new_time;
    }

    size_t curr_step{0};
    double curr_time{0.0};

    gtsam::Pose3 prev_pose;
    gtsam::Pose3 curr_pose;
    gtsam::Matrix66 prev_cov{gtsam::Matrix66::Identity()};
    gtsam::Matrix66 curr_cov{gtsam::Matrix66::Identity()};
  };

  // --- Initialization ---
  auto computeInitialState(double init_time, utils::QueueBundle const& queues) const
      -> std::optional<InitialState>;

  auto computeInitialOrientation(std::shared_ptr<utils::AhrsData> const& ahrs) const -> gtsam::Rot3;

  auto computeInitialPosition(gtsam::Rot3 const& map_R_target,
                              std::shared_ptr<utils::OdometryData> const& gps,
                              std::shared_ptr<utils::OdometryData> const& depth) const
      -> gtsam::Point3;

  auto computeInitialVelocity(gtsam::Rot3 const& map_R_target,
                              std::shared_ptr<utils::TwistData> const& dvl) const -> gtsam::Vector3;

  auto computeInitialPoseCovariance(gtsam::Rot3 const& map_R_target,
                                    std::shared_ptr<utils::OdometryData> const& gps,
                                    std::shared_ptr<utils::OdometryData> const& depth,
                                    std::shared_ptr<utils::AhrsData> const& ahrs) const
      -> gtsam::Matrix6;

  auto computeInitialVelocityCovariance(gtsam::Rot3 const& map_R_target,
                                        std::shared_ptr<utils::TwistData> const& dvl,
                                        gtsam::Matrix3 const& target_orientation_cov) const
      -> gtsam::Matrix3;

  // --- Configuration ---
  auto configureImuPreintegration(InitialState const& init_state) const
      -> std::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params>;

  // --- Factor Construction ---
  void addPriorFactors(InitialState const& init_state, gtsam::NonlinearFactorGraph& graph,
                       gtsam::Values& values);

  void addGpsFactor(gtsam::NonlinearFactorGraph& graph,
                    std::deque<std::shared_ptr<utils::OdometryData>> const& gps_msgs);

  void addDepthFactor(gtsam::NonlinearFactorGraph& graph,
                      std::deque<std::shared_ptr<utils::OdometryData>> const& depth_msgs);

  void addMagFactor(gtsam::NonlinearFactorGraph& graph,
                    std::deque<std::shared_ptr<utils::MagneticFieldData>> const& mag_msgs);

  void addAhrsFactor(gtsam::NonlinearFactorGraph& graph,
                     std::deque<std::shared_ptr<utils::AhrsData>> const& ahrs_msgs);

  void addDvlFactor(gtsam::NonlinearFactorGraph& graph,
                    std::deque<std::shared_ptr<utils::TwistData>> const& dvl_msgs,
                    gtsam::Vector3 const& imu_gyro);

  void addConstVelFactor(gtsam::NonlinearFactorGraph& graph, double target_time);

  void addWrenchDynamicsFactor(gtsam::NonlinearFactorGraph& graph,
                               std::deque<std::shared_ptr<utils::WrenchData>> const& wrench_msgs,
                               double target_time);

  void addImuPreintFactor(gtsam::NonlinearFactorGraph& graph,
                          std::deque<std::shared_ptr<utils::ImuData>> const& imu_msgs,
                          double target_time);

  void addDvlLoosePreintFactor(gtsam::NonlinearFactorGraph& graph,
                               std::deque<std::shared_ptr<utils::TwistData>> const& dvl_msgs,
                               std::deque<std::shared_ptr<utils::AhrsData>> const& ahrs_msgs,
                               double target_time);

  void addDvlTightPreintFactor(gtsam::NonlinearFactorGraph& graph,
                               std::deque<std::shared_ptr<utils::TwistData>> const& dvl_msgs,
                               std::deque<std::shared_ptr<utils::ImuData>> const& imu_msgs,
                               double target_time, gtsam::Vector3 const& held_imu_accel,
                               gtsam::Vector3 const& held_imu_gyro);

  void addOriginDeltaPriorFactor(gtsam::NonlinearFactorGraph& graph, gtsam::Values& values,
                                 size_t agent_queue_idx, utils::AgentStatusData const& msg);

  void addNeighborPriorFactor(gtsam::NonlinearFactorGraph& graph, NeighborState const& neighbor,
                              size_t agent_queue_idx);

  static void addNeighborBetweenFactor(gtsam::NonlinearFactorGraph& graph,
                                       NeighborState const& neighbor);

  void addNeighborDepthFactor(gtsam::NonlinearFactorGraph& graph, utils::AgentStatusData const& msg,
                              NeighborState const& neighbor, size_t agent_queue_idx);

  void addNeighborAhrsFactor(gtsam::NonlinearFactorGraph& graph, utils::AgentStatusData const& msg,
                             NeighborState const& neighbor, size_t agent_queue_idx);

  void addInterAgentRangeFactor(gtsam::NonlinearFactorGraph& graph,
                                utils::AgentStatusData const& msg, NeighborState const& neighbor,
                                gtsam::Key pose_key, size_t agent_queue_idx);

  void addInterAgentBearingFactor(gtsam::NonlinearFactorGraph& graph,
                                  utils::AgentStatusData const& msg, NeighborState const& neighbor,
                                  gtsam::Key pose_key, size_t agent_queue_idx);

  void addMultiAgentFactors(gtsam::NonlinearFactorGraph& graph, gtsam::Values& values,
                            gtsam::IncrementalFixedLagSmoother::KeyTimestampMap& timestamps,
                            utils::QueueBundle const& queues, double target_time);

  // --- Parameters ---
  factor_graph_node::Params const params_;
  utils::TfBundle tfs_;

  // --- Logging ---
  utils::Logger logger_;

  // --- GTSAM Solver ---
  std::unique_ptr<gtsam::IncrementalFixedLagSmoother> inc_smoother_;
  std::unique_ptr<gtsam::ISAM2> isam_;
  gtsam::NonlinearFactorGraph lm_graph_;
  gtsam::Values lm_values_;
  std::unique_ptr<gtsam::PreintegratedCombinedMeasurements> imu_preintegrator_;
  std::unique_ptr<utils::DvlLoosePreintegrator> dvl_loose_preintegrator_;
  std::unique_ptr<utils::DvlTightPreintegrator> dvl_tight_preintegrator_;

  // --- State Estimates ---
  size_t prev_step_{0};
  size_t curr_step_{1};
  double prev_time_{0.0};

  gtsam::Pose3 prev_pose_;
  gtsam::Vector3 prev_vel_;
  gtsam::imuBias::ConstantBias prev_imu_bias_;
  gtsam::Point3 prev_mag_bias_;

  // --- Multi-agent ---
  gtsam::Pose3 neighbor_base_T_modem_;
  std::unordered_map<size_t, NeighborState> neighbors_;
  std::unordered_map<size_t, gtsam::Pose3> prev_origin_deltas_;

  // --- Sensor Data ---
  gtsam::Vector3 last_dvl_vel_{gtsam::Vector3::Zero()};
  gtsam::Matrix3 last_dvl_cov_{gtsam::Matrix3::Zero()};
  gtsam::Vector3 last_imu_accel_{gtsam::Vector3::Zero()};
  gtsam::Vector3 last_imu_gyro_{gtsam::Vector3::Zero()};
  std::shared_ptr<utils::WrenchData> last_wrench_msg_;

  // --- Buffer ---
  mutable std::mutex state_mutex_;
  std::map<int64_t, gtsam::Key> time_to_key_;
  gtsam::NonlinearFactorGraph buffer_graph_;
  gtsam::Values buffer_values_;
  gtsam::IncrementalFixedLagSmoother::KeyTimestampMap buffer_timestamps_;
  double buffer_target_time_{0.0};
  size_t buffer_prev_step_{0};
  size_t buffer_keyframes_{0};
  bool has_buffer_{false};
};

}  // namespace coug_fg
