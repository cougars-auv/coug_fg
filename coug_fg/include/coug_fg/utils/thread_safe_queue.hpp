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

#include <chrono>
#include <deque>
#include <mutex>
#include <optional>
#include <utility>

namespace coug_fg::utils {

template <typename T>
class ThreadSafeQueue {
 public:
  void push(T const& value) {
    std::scoped_lock const lock(mutex_);
    queue_.push_back(value);
    last_msg_time_ = value->timestamp;
    last_arrival_time_ = std::chrono::steady_clock::now();
  }

  auto drain() -> std::deque<T> {
    std::scoped_lock const lock(mutex_);
    return std::exchange(queue_, {});
  }

  auto empty() const -> bool {
    std::scoped_lock const lock(mutex_);
    return queue_.empty();
  }

  auto size() const -> size_t {
    std::scoped_lock const lock(mutex_);
    return queue_.size();
  }

  auto getLastTime() const -> std::optional<double> {
    std::scoped_lock const lock(mutex_);
    return last_msg_time_;
  }

  auto secondsSinceLastArrival() const -> std::optional<double> {
    std::scoped_lock const lock(mutex_);
    if (!last_arrival_time_.has_value()) {
      return std::nullopt;
    }
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - *last_arrival_time_)
        .count();
  }

  void restore(std::deque<T> const& items) {
    std::scoped_lock const lock(mutex_);
    queue_.insert(queue_.begin(), items.begin(), items.end());
  }

 private:
  mutable std::mutex mutex_;
  std::deque<T> queue_;
  std::optional<double> last_msg_time_;
  std::optional<std::chrono::steady_clock::time_point> last_arrival_time_;
};

}  // namespace coug_fg::utils
