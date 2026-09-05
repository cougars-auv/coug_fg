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

#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace coug_fg::utils {

enum class LogLevel : std::uint8_t { kDebug, kInfo, kWarn, kError };

using LogCallback = std::function<void(LogLevel, const std::string&)>;

class Logger {
 public:
  void setCallback(LogCallback callback) { callback_ = std::move(callback); }

  void log(LogLevel level, const std::string& msg) const {
    if (callback_) {
      callback_(level, msg);
    }
  }

  void logOnce(LogLevel level, const std::string& key, const std::string& msg) const {
    {
      std::scoped_lock lock(mutex_);
      if (!once_keys_.insert(key).second) {
        return;
      }
    }
    log(level, msg);
  }

  void logThrottled(LogLevel level, const std::string& key, double period, double now,
                    const std::string& msg) const {
    {
      std::scoped_lock lock(mutex_);
      auto it = last_emit_time_.find(key);
      if (it != last_emit_time_.end() && now - it->second < period) {
        return;
      }
      last_emit_time_[key] = now;
    }
    log(level, msg);
  }

 private:
  LogCallback callback_;
  mutable std::mutex mutex_;
  mutable std::unordered_set<std::string> once_keys_;
  mutable std::unordered_map<std::string, double> last_emit_time_;
};

}  // namespace coug_fg::utils
