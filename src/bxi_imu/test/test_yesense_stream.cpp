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

#include <pty.h>
#include <unistd.h>

#include <array>
#include <cstdint>
#include <vector>

#include "yesense_backend.hpp"

namespace
{

void append_i32(std::vector<std::uint8_t> & bytes, std::int32_t value)
{
  for (int shift = 0; shift < 32; shift += 8) {
    bytes.push_back(static_cast<std::uint8_t>(value >> shift));
  }
}

std::vector<std::uint8_t> make_frame(std::int32_t acceleration_x, std::uint16_t tid)
{
  std::vector<std::uint8_t> payload;
  const auto append_axis = [&payload](std::uint8_t id, std::int32_t x) {
      payload.push_back(id);
      payload.push_back(12);
      append_i32(payload, x);
      append_i32(payload, 0);
      append_i32(payload, 0);
    };
  append_axis(0x10, acceleration_x);
  append_axis(0x20, 0);
  payload.push_back(0x41);
  payload.push_back(16);
  append_i32(payload, 1000000);
  append_i32(payload, 0);
  append_i32(payload, 0);
  append_i32(payload, 0);

  std::vector<std::uint8_t> frame = {
    0x59, 0x53, static_cast<std::uint8_t>(tid),
    static_cast<std::uint8_t>(tid >> 8), static_cast<std::uint8_t>(payload.size())};
  frame.reserve(frame.size() + payload.size() + 2);
  for (const auto byte : payload) {
    frame.push_back(byte);
  }
  std::uint8_t ck1 = 0;
  std::uint8_t ck2 = 0;
  for (std::size_t index = 2; index < frame.size(); ++index) {
    ck1 = static_cast<std::uint8_t>(ck1 + frame[index]);
    ck2 = static_cast<std::uint8_t>(ck2 + ck1);
  }
  frame.push_back(ck1);
  frame.push_back(ck2);
  return frame;
}

class YesenseStreamTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    ASSERT_EQ(openpty(&master_, &slave_, path_.data(), nullptr, nullptr), 0);
    close(slave_);
    slave_ = -1;
  }

  void TearDown() override
  {
    if (master_ >= 0) {
      close(master_);
    }
  }

  void write_bytes(const std::vector<std::uint8_t> & bytes)
  {
    ASSERT_EQ(::write(master_, bytes.data(), bytes.size()),
      static_cast<ssize_t>(bytes.size()));
  }

  std::array<char, 128> path_{};
  int master_{-1};
  int slave_{-1};
};

TEST_F(YesenseStreamTest, ReturnsNewestFrameFromOneSerialRead)
{
  bxi_imu::YesenseBackend backend(path_.data(), 921600, rclcpp::get_logger("yesense_test"));
  ASSERT_TRUE(backend.open());
  std::vector<std::uint8_t> bytes;
  for (int index = 1; index <= 3; ++index) {
    const auto frame = make_frame(index * 1000000, index);
    bytes.insert(bytes.end(), frame.begin(), frame.end());
  }
  write_bytes(bytes);

  bxi_imu::ImuSample sample;
  ASSERT_TRUE(backend.read(sample));
  EXPECT_NEAR(sample.imu.linear_acceleration.x, 3.0, 1e-6);
  ASSERT_TRUE(sample.device_frame_id.has_value());
  EXPECT_EQ(*sample.device_frame_id, 3u);
}

TEST_F(YesenseStreamTest, KeepsFrameSplitAcrossSerialReads)
{
  bxi_imu::YesenseBackend backend(path_.data(), 921600, rclcpp::get_logger("yesense_test"));
  ASSERT_TRUE(backend.open());
  const auto frame = make_frame(2000000, 4);
  write_bytes(std::vector<std::uint8_t>(frame.begin(), frame.begin() + 20));
  bxi_imu::ImuSample sample;
  EXPECT_FALSE(backend.read(sample));
  write_bytes(std::vector<std::uint8_t>(frame.begin() + 20, frame.end()));
  ASSERT_TRUE(backend.read(sample));
  EXPECT_NEAR(sample.imu.linear_acceleration.x, 2.0, 1e-6);
}

TEST_F(YesenseStreamTest, ProcessesBatchLargerThanDecoderBufferChunk)
{
  bxi_imu::YesenseBackend backend(path_.data(), 921600, rclcpp::get_logger("yesense_test"));
  ASSERT_TRUE(backend.open());
  std::vector<std::uint8_t> bytes;
  for (int index = 1; index <= 12; ++index) {
    const auto frame = make_frame(index * 1000000, index);
    bytes.insert(bytes.end(), frame.begin(), frame.end());
  }
  write_bytes(bytes);

  bxi_imu::ImuSample sample;
  ASSERT_TRUE(backend.read(sample));
  EXPECT_NEAR(sample.imu.linear_acceleration.x, 12.0, 1e-6);
}

TEST_F(YesenseStreamTest, SkipsBadCrcAndReadsFollowingValidFrame)
{
  bxi_imu::YesenseBackend backend(path_.data(), 921600, rclcpp::get_logger("yesense_test"));
  ASSERT_TRUE(backend.open());
  auto bytes = make_frame(1000000, 1);
  bytes.back() ^= 0xff;
  const auto good = make_frame(4000000, 2);
  bytes.insert(bytes.end(), good.begin(), good.end());
  write_bytes(bytes);

  bxi_imu::ImuSample sample;
  ASSERT_TRUE(backend.read(sample));
  EXPECT_NEAR(sample.imu.linear_acceleration.x, 4.0, 1e-6);
}

}  // namespace
