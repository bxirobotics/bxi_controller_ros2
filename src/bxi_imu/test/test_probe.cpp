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

#include <gtest/gtest.h>

#include <chrono>
#include <limits>
#include <thread>
#include <utility>
#include <vector>

#include "bxi_imu/probe.hpp"

namespace
{

class FakeBackend : public bxi_imu::ImuBackend
{
public:
  explicit FakeBackend(std::vector<bxi_imu::ImuSample> samples, bool disconnect = false)
  : samples_(std::move(samples)), disconnect_(disconnect) {}

  bool open() override {return open_;}
  bool read(bxi_imu::ImuSample & sample) override
  {
    if (next_ < samples_.size()) {
      sample = samples_[next_++];
      return true;
    }
    if (disconnect_) {
      open_ = false;
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return false;
  }
  bool is_open() const override {return open_;}
  void close() override {open_ = false;}
  std::string name() const override {return "fake";}

private:
  std::vector<bxi_imu::ImuSample> samples_;
  std::size_t next_{0};
  bool disconnect_{false};
  bool open_{true};
};

bxi_imu::ImuSample valid_sample()
{
  bxi_imu::ImuSample sample;
  sample.imu.orientation.w = 1.0;
  return sample;
}

TEST(ImuProbe, SelectsOnlyAfterConsecutiveValidFrames)
{
  auto invalid = valid_sample();
  invalid.imu.orientation.w = 0.0;
  FakeBackend backend({valid_sample(), invalid, valid_sample(), valid_sample(), valid_sample()});
  const auto result = bxi_imu::probe_backend(backend, std::chrono::milliseconds(20), 3, 0.1);
  EXPECT_TRUE(result.matched);
  EXPECT_EQ(result.valid_frames, 4);
  EXPECT_EQ(result.invalid_frames, 1);
}

TEST(ImuProbe, RejectsFramesWithNonFiniteMotion)
{
  auto invalid = valid_sample();
  invalid.imu.angular_velocity.x = std::numeric_limits<double>::infinity();
  FakeBackend backend({invalid}, true);
  const auto result = bxi_imu::probe_backend(backend, std::chrono::milliseconds(20), 3, 0.1);
  EXPECT_FALSE(result.matched);
  EXPECT_TRUE(result.device_lost);
  EXPECT_EQ(result.invalid_frames, 1);
}

TEST(ImuProbe, RejectsWrongProtocolWithoutValidFrames)
{
  FakeBackend backend({});
  const auto result = bxi_imu::probe_backend(backend, std::chrono::milliseconds(20), 3, 0.1);
  EXPECT_FALSE(result.matched);
  EXPECT_FALSE(result.device_lost);
  EXPECT_EQ(result.valid_frames, 0);
}

}  // namespace
