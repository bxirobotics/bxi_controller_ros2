// Copyright 2026 BXI Robotics
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
#include <cmath>

#include "bxi_imu/imu_backend.hpp"

namespace bxi_imu
{

struct ProbeResult
{
  bool matched{false};
  bool device_lost{false};
  int valid_frames{0};
  int invalid_frames{0};
};

inline bool valid_probe_sample(const ImuSample & sample, double quaternion_tolerance)
{
  const auto & q = sample.imu.orientation;
  const auto & gyro = sample.imu.angular_velocity;
  const auto & acc = sample.imu.linear_acceleration;
  const double norm = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
  return std::isfinite(norm) &&
         norm >= 1.0 - quaternion_tolerance && norm <= 1.0 + quaternion_tolerance &&
         std::isfinite(gyro.x) && std::isfinite(gyro.y) && std::isfinite(gyro.z) &&
         std::isfinite(acc.x) && std::isfinite(acc.y) && std::isfinite(acc.z);
}

inline ProbeResult probe_backend(
  ImuBackend & backend, std::chrono::milliseconds timeout, int required_frames,
  double quaternion_tolerance)
{
  ProbeResult result;
  int consecutive_frames = 0;
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    ImuSample sample;
    if (backend.read(sample)) {
      if (valid_probe_sample(sample, quaternion_tolerance)) {
        ++result.valid_frames;
        if (++consecutive_frames >= required_frames) {
          result.matched = true;
          return result;
        }
      } else {
        ++result.invalid_frames;
        consecutive_frames = 0;
      }
    } else if (!backend.is_open()) {
      result.device_lost = true;
      return result;
    }
  }
  return result;
}

}  // namespace bxi_imu
