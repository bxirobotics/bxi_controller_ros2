// Copyright 2026 BXI Robotics
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <cstdlib>
#include <future>

// Exercise the actual parameter parser without exposing it as a public API.
#include "../src/device_setup.cpp"  // NOLINT(build/include)

namespace bxi_imu
{
namespace
{

class DeviceSetupTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    master_ = posix_openpt(O_RDWR | O_NOCTTY);
    ASSERT_GE(master_, 0);
    ASSERT_EQ(grantpt(master_), 0);
    ASSERT_EQ(unlockpt(master_), 0);
    ASSERT_NE(ptsname(master_), nullptr);
    path_ = ptsname(master_);
  }

  void TearDown() override
  {
    if (master_ >= 0) {::close(master_);}
  }

  std::string exchange(Serial & serial, const std::string & command, const Bytes & reply)
  {
    auto device = std::async(
      std::launch::async, [&]() {
        std::string request;
        const auto deadline = Clock::now() + std::chrono::seconds(1);
        while (request.find('\n') == std::string::npos && Clock::now() < deadline) {
          pollfd p{master_, POLLIN, 0};
          if (poll(&p, 1, 20) <= 0) {continue;}
          char data[128];
          const auto count = ::read(master_, data, sizeof(data));
          if (count <= 0) {return false;}
          request.append(data, size_t(count));
        }
        return request == command + "\r\n" &&
        ::write(master_, reply.data(), reply.size()) == ssize_t(reply.size());
      });
    const auto result = hip_request(serial, command, false, 500);
    EXPECT_TRUE(device.get());
    return result;
  }

  static Bytes packet()
  {
    // Fake ACKs inside binary payloads must be skipped even when the packet spans commands.
    const std::string payload = "\nOK\r\nERR\nBAUD=1\n>";
    Bytes result{0x5a, 0xa5, uint8_t(payload.size()), 0, 0, 0};
    result.insert(result.end(), payload.begin(), payload.end());
    return result;
  }

  static Bytes response(const std::string & text)
  {
    return Bytes(text.begin(), text.end());
  }

  int master_{-1};
  std::string path_;
};

TEST_F(DeviceSetupTest, KeepsPartialTelemetryAcrossCommands)
{
  Serial serial(path_);
  serial.set_baud(921600);
  const auto frame = packet();
  for (size_t split = 1; split < frame.size(); ++split) {
    SCOPED_TRACE(split);
    auto first = response("BAUD=921600\nOK\r\n");
    first.insert(first.end(), frame.begin(), frame.begin() + split);
    EXPECT_EQ(exchange(serial, "LOG COMCONFIG", first), "BAUD=921600\n");
    Bytes second(frame.begin() + split, frame.end());
    const auto ack = response("OK\r\n");
    second.insert(second.end(), ack.begin(), ack.end());
    EXPECT_TRUE(exchange(serial, "LOG ENABLE", second).empty());
  }
}

TEST_F(DeviceSetupTest, RejectsBinaryAckWithoutRealReply)
{
  Serial serial(path_);
  serial.set_baud(921600);
  EXPECT_THROW(exchange(serial, "LOG ENABLE", packet()), std::runtime_error);
}

TEST_F(DeviceSetupTest, AcceptsAckAfterCapturedHi12s2FrameTail)
{
  // HI12S2-MI1-000 telemetry captured on elf3-141; the final byte is ASCII '>'.
  const Bytes frame{
    0x5a, 0xa5, 0x4c, 0x00, 0xd9, 0x1b, 0x91, 0x00, 0x1a, 0x1e, 0x00, 0x00,
    0x00, 0x00, 0xcf, 0xa6, 0x1b, 0x00, 0x85, 0x52, 0x7c, 0x3f, 0x67, 0x8f,
    0x00, 0x3c, 0x3f, 0xfd, 0x22, 0x3e, 0xbd, 0xbd, 0x61, 0x3c, 0x55, 0xe8,
    0xcb, 0xbc, 0x7a, 0x31, 0x0c, 0xbd, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x15, 0x98, 0xa1, 0xc2, 0xfc, 0x8f,
    0xeb, 0x3e, 0x3a, 0xd4, 0x34, 0xc2, 0xa7, 0xb9, 0x33, 0xbf, 0x6f, 0x2f,
    0x7c, 0x3e, 0xfa, 0x77, 0x19, 0x3f, 0x37, 0x18, 0x97, 0x3e};
  Serial serial(path_);
  serial.set_baud(921600);
  auto first = response("BAUD=921600\nOK\r\n");
  first.insert(first.end(), frame.begin(), frame.begin() + 37);
  EXPECT_EQ(exchange(serial, "LOG COMCONFIG", first), "BAUD=921600\n");
  Bytes second(frame.begin() + 37, frame.end());
  const auto ack = response("OK\r\n");
  second.insert(second.end(), ack.begin(), ack.end());
  EXPECT_TRUE(exchange(serial, "LOG ENABLE", second).empty());
}

TEST_F(DeviceSetupTest, PreservesExplicitDeviceRejection)
{
  Serial serial(path_);
  serial.set_baud(921600);
  auto first = response("OK\r\n");
  const auto frame = packet();
  first.insert(first.end(), frame.begin(), frame.end() - 1);
  EXPECT_TRUE(exchange(serial, "LOG COMCONFIG", first).empty());
  try {
    exchange(serial, "LOG ENABLE", response(">ERR\r\n"));
    FAIL() << "device rejection was accepted";
  } catch (const std::runtime_error & error) {
    EXPECT_EQ(std::string(error.what()), "HiPNUC rejected: LOG ENABLE");
  }
}

TEST_F(DeviceSetupTest, ResetsPartialTelemetryWhenBaudChanges)
{
  Serial serial(path_);
  serial.set_baud(115200);
  EXPECT_TRUE(exchange(serial, "LOG VERSION", response("OK\r\n\x5a")).empty());
  serial.set_baud(921600);
  EXPECT_EQ(
    exchange(serial, "LOG VERSION", response("PNAME=HI12S2\nOK\r\n")),
    "PNAME=HI12S2\n");
}

}  // namespace
}  // namespace bxi_imu
