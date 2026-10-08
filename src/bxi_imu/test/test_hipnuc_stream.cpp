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
#include <cstring>
#include <vector>

#include "hipnuc_backend.hpp"

namespace
{

std::uint16_t update_crc(std::uint16_t crc, std::uint8_t byte)
{
  crc ^= static_cast<std::uint16_t>(byte) << 8;
  for (int bit = 0; bit < 8; ++bit) {
    crc = static_cast<std::uint16_t>((crc << 1) ^ ((crc & 0x8000) ? 0x1021 : 0));
  }
  return crc;
}

std::vector<std::uint8_t> make_packet(const std::vector<std::uint8_t> & payload)
{
  const auto size = static_cast<std::uint16_t>(payload.size());
  std::vector<std::uint8_t> packet = {
    0x5a, 0xa5, static_cast<std::uint8_t>(size), static_cast<std::uint8_t>(size >> 8), 0, 0};
  packet.reserve(packet.size() + payload.size());
  for (const auto byte : payload) {
    packet.push_back(byte);
  }
  std::uint16_t crc = 0;
  for (std::size_t index = 0; index < packet.size(); ++index) {
    if (index != 4 && index != 5) {
      crc = update_crc(crc, packet[index]);
    }
  }
  packet[4] = static_cast<std::uint8_t>(crc);
  packet[5] = static_cast<std::uint8_t>(crc >> 8);
  return packet;
}

std::vector<std::uint8_t> make_imu_frame(float acceleration_x)
{
  hi91_t data{};
  data.tag = 0x91;
  data.system_time = static_cast<std::uint32_t>(acceleration_x * 10);
  data.acc[0] = acceleration_x;
  data.quat[0] = 1.0f;
  std::vector<std::uint8_t> payload(sizeof(data));
  std::memcpy(payload.data(), &data, sizeof(data));
  return make_packet(payload);
}

class HipnucStreamTest : public ::testing::Test
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

TEST_F(HipnucStreamTest, ReturnsNewestFrameFromOneSerialRead)
{
  bxi_imu::HipnucBackend backend(path_.data(), 921600, rclcpp::get_logger("hipnuc_test"));
  ASSERT_TRUE(backend.open());
  std::vector<std::uint8_t> bytes;
  for (int index = 1; index <= 3; ++index) {
    const auto frame = make_imu_frame(static_cast<float>(index));
    bytes.insert(bytes.end(), frame.begin(), frame.end());
  }
  write_bytes(bytes);

  bxi_imu::ImuSample sample;
  ASSERT_TRUE(backend.read(sample));
  EXPECT_NEAR(sample.imu.linear_acceleration.x, 3.0 * 9.8, 1e-5);
  ASSERT_TRUE(sample.device_tick.has_value());
  EXPECT_EQ(*sample.device_tick, 30u);
  EXPECT_EQ(sample.device_tick_period_us, 1000u);
}

TEST_F(HipnucStreamTest, KeepsFrameSplitAcrossSerialReads)
{
  bxi_imu::HipnucBackend backend(path_.data(), 921600, rclcpp::get_logger("hipnuc_test"));
  ASSERT_TRUE(backend.open());
  const auto frame = make_imu_frame(2.0f);
  write_bytes(std::vector<std::uint8_t>(frame.begin(), frame.begin() + 20));
  bxi_imu::ImuSample sample;
  EXPECT_FALSE(backend.read(sample));
  write_bytes(std::vector<std::uint8_t>(frame.begin() + 20, frame.end()));
  ASSERT_TRUE(backend.read(sample));
  EXPECT_NEAR(sample.imu.linear_acceleration.x, 2.0 * 9.8, 1e-5);
}

TEST_F(HipnucStreamTest, DrainsMoreThanOneKernelRead)
{
  bxi_imu::HipnucBackend backend(path_.data(), 921600, rclcpp::get_logger("hipnuc_test"));
  ASSERT_TRUE(backend.open());
  std::vector<std::uint8_t> bytes;
  for (int index = 1; index <= 20; ++index) {
    const auto frame = make_imu_frame(static_cast<float>(index));
    bytes.insert(bytes.end(), frame.begin(), frame.end());
  }
  ASSERT_GT(bytes.size(), 1024u);
  write_bytes(bytes);
  bxi_imu::ImuSample sample;
  ASSERT_TRUE(backend.read(sample));
  EXPECT_NEAR(sample.imu.linear_acceleration.x, 20.0 * 9.8, 1e-5);
}

TEST_F(HipnucStreamTest, IgnoresBadCrcAndUnsupportedTrailingFrame)
{
  bxi_imu::HipnucBackend backend(path_.data(), 921600, rclcpp::get_logger("hipnuc_test"));
  ASSERT_TRUE(backend.open());
  auto bytes = make_imu_frame(1.0f);
  bytes[4] ^= 0xff;
  const auto good = make_imu_frame(4.0f);
  bytes.insert(bytes.end(), good.begin(), good.end());
  const auto unsupported = make_packet({0x42});
  bytes.insert(bytes.end(), unsupported.begin(), unsupported.end());
  write_bytes(bytes);

  bxi_imu::ImuSample sample;
  ASSERT_TRUE(backend.read(sample));
  EXPECT_NEAR(sample.imu.linear_acceleration.x, 4.0 * 9.8, 1e-5);
}

}  // namespace
