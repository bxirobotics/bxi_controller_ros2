// Copyright 2026 BXI Robotics
// SPDX-License-Identifier: Apache-2.0
// Parameter protocol adapted from play/imu-tool/native/imu_tool.cpp.
#include "bxi_imu/device_setup.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <poll.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <asm/termbits.h>
#include <asm/ioctls.h>
#include <rclcpp/logging.hpp>

namespace bxi_imu
{
namespace
{
using Bytes = std::vector<uint8_t>;
using Clock = std::chrono::steady_clock;

void require(bool valid, const std::string & reason)
{
  if (!valid) {throw std::runtime_error(reason);}
}

void pause_ms(int ms)
{
  std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

uint16_t u16(const uint8_t * p) {return p[0] | (uint16_t(p[1]) << 8);}

std::array<uint8_t, 2> checksum(const uint8_t * p, size_t length)
{
  uint8_t a = 0, b = 0;
  for (size_t i = 0; i < length; ++i) {
    a += p[i]; b += a;
  }
  return {a, b};
}

Bytes yes_command(int cls, int op, const Bytes & payload)
{
  const size_t word = (payload.size() << 3) | op;
  Bytes bytes{0x59, 0x53, uint8_t(cls), uint8_t(word), uint8_t(word >> 8)};
  bytes.reserve(payload.size() + 7);
  bytes.insert(bytes.end(), payload.begin(), payload.end());
  const auto ck = checksum(bytes.data() + 2, bytes.size() - 2);
  bytes.insert(bytes.end(), ck.begin(), ck.end());
  return bytes;
}

bool valid_yes(const Bytes & b, size_t at, size_t length)
{
  if (at + length > b.size()) {return false;}
  const auto ck = checksum(b.data() + at + 2, length - 4);
  return ck[0] == b[at + length - 2] && ck[1] == b[at + length - 1];
}

bool take_yes(Bytes & b, Bytes & payload, int cls, int op)
{
  size_t keep = b.size();
  for (size_t i = 0; i < b.size(); ++i) {
    if (b[i] != 0x59) {continue;}
    if (i + 5 > b.size()) {keep = std::min(keep, i); continue;}
    if (b[i + 1] != 0x53) {continue;}
    const size_t output_length = size_t(b[i + 4]) + 7;
    const size_t param_length = size_t(u16(b.data() + i + 3) >> 3) + 7;
    const bool param_ok = valid_yes(b, i, param_length);
    if (param_ok && b[i + 2] == cls && (b[i + 3] & 7) == op) {
      payload.assign(b.begin() + i + 5, b.begin() + i + param_length - 2);
      b.erase(b.begin(), b.begin() + i + param_length);
      return true;
    }
    // Telemetry and parameter frames share a sync word but use different lengths.
    const bool output_ok = valid_yes(b, i, output_length);
    if (output_ok || param_ok) {
      i += (output_ok ? output_length : param_length) - 1;
    } else if (i + param_length > b.size()) {
      keep = std::min(keep, i);
    }
  }
  b.erase(b.begin(), b.begin() + keep);
  return false;
}

class Serial
{
public:
  explicit Serial(const std::string & port)
  : fd_(::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC))
  {
    require(fd_ >= 0, "open " + port + ": " + std::strerror(errno));
    try {
      int exclusive = 0;
      require(
        ioctl(fd_, TIOCGEXCL, &exclusive) == 0 && exclusive == 0,
        "IMU port is exclusively owned: " + port);
      require(
        flock(fd_, LOCK_EX | LOCK_NB) == 0 && ioctl(fd_, TIOCEXCL) == 0,
        "IMU port is busy: " + port);
    } catch (...) {
      ::close(fd_);
      throw;
    }
  }
  ~Serial() {ioctl(fd_, TIOCNXCL); ::close(fd_);}
  Serial(const Serial &) = delete;
  Serial & operator=(const Serial &) = delete;

  void set_baud(int baud)
  {
    termios2 settings{};
    require(ioctl(fd_, TCGETS2, &settings) == 0, "TCGETS2 failed");
    settings.c_iflag = 0; settings.c_oflag = 0; settings.c_lflag = 0;
    settings.c_cflag = BOTHER | CS8 | CREAD | CLOCAL;
    settings.c_ispeed = settings.c_ospeed = baud;
    settings.c_cc[VMIN] = 0; settings.c_cc[VTIME] = 0;
    require(ioctl(fd_, TCSETS2, &settings) == 0, "TCSETS2 failed");
    clear();
  }
  void clear() {require(ioctl(fd_, TCFLSH, 0) == 0, "IMU input flush failed");}
  Bytes read_chunk()
  {
    pollfd p{fd_, POLLIN, 0};
    const int ready = poll(&p, 1, 40);
    if (ready < 0 && errno == EINTR) {return {};}
    require(
      ready >= 0 && !(p.revents & (POLLERR | POLLHUP | POLLNVAL)),
      "IMU disconnected during setup");
    if (!ready) {return {};}
    uint8_t bytes[4096];
    const auto length = ::read(fd_, bytes, sizeof(bytes));
    if (length < 0 && (errno == EAGAIN || errno == EINTR)) {return {};}
    require(length > 0, "IMU setup read failed");
    return Bytes(bytes, bytes + length);
  }
  void write_all(const Bytes & bytes)
  {
    const auto deadline = Clock::now() + std::chrono::seconds(1);
    size_t offset = 0;
    while (offset < bytes.size()) {
      require(Clock::now() < deadline, "IMU setup write timed out");
      const auto count = ::write(fd_, bytes.data() + offset, bytes.size() - offset);
      if (count > 0) {offset += size_t(count);} else {
        require(count < 0 && (errno == EAGAIN || errno == EINTR), "IMU setup write failed");
        pollfd p{fd_, POLLOUT, 0};
        poll(&p, 1, 20);
      }
    }
  }

private:
  int fd_;
};

Bytes yes_request(
  Serial & serial, int cls, int op = 0, const Bytes & payload = {}, int timeout_ms = 500)
{
  serial.clear();
  serial.write_all(yes_command(cls, op, payload));
  Bytes buffer, result;
  const auto deadline = Clock::now() + std::chrono::milliseconds(timeout_ms);
  while (Clock::now() < deadline) {
    const auto chunk = serial.read_chunk();
    buffer.insert(buffer.end(), chunk.begin(), chunk.end());
    if (take_yes(buffer, result, cls, op)) {return result;}
  }
  throw std::runtime_error("Yesense parameter reply missing: class=" + std::to_string(cls));
}

std::string hip_request(
  Serial & serial, const std::string & command, bool allow_silent = false, int timeout_ms = 500)
{
  serial.clear();
  const std::string wire = command + "\r\n";
  serial.write_all(Bytes(wire.begin(), wire.end()));
  std::string line, lines;
  Bytes buffer;
  const auto deadline = Clock::now() + std::chrono::milliseconds(timeout_ms);
  while (Clock::now() < deadline) {
    const auto chunk = serial.read_chunk();
    buffer.insert(buffer.end(), chunk.begin(), chunk.end());
    // Binary payloads (even corrupt ones) must never impersonate an ASCII ACK.
    while (!buffer.empty()) {
      if (buffer[0] == 0x5a) {
        if (buffer.size() < 2) {break;}
        if (buffer[1] == 0xa5) {
          if (buffer.size() < 6) {break;}
          const size_t length = u16(buffer.data() + 2) + 6;
          if (length > 6 && length <= 512) {
            if (buffer.size() < length) {break;}
            buffer.erase(buffer.begin(), buffer.begin() + length);
            line.clear();
            continue;
          }
        }
      }
      const uint8_t c = buffer.front();
      buffer.erase(buffer.begin());
      if (c == '\n') {
        require(line != "ERR", "HiPNUC rejected: " + command);
        if (line == "OK") {return lines;}
        if (line.find('=') != std::string::npos) {lines += line + "\n";}
        line.clear();
      } else if (c == '\r') {
        continue;
      } else if (c >= 32 && c <= 126 && line.size() < 256) {
        line += char(c);
      } else {line.clear();}
    }
  }
  require(allow_silent, "HiPNUC parameter reply missing: " + command);
  return lines;
}

std::string field(const std::string & lines, const std::string & key)
{
  const std::string marker = key + "=";
  size_t at = lines.find(marker);
  while (at != std::string::npos && at > 0 && lines[at - 1] != '\n') {
    at = lines.find(marker, at + 1);
  }
  if (at == std::string::npos) {return {};}
  at += marker.size();
  return lines.substr(at, lines.find('\n', at) - at);
}

bool hip_target(const std::string & config)
{
  return field(config, "BAUD") == "921600" &&
         (field(config, "HI91") == "2" || field(config, "HI91") == "ONTIME=2" ||
         field(config, "HI91, ONTIME") == "2" || field(config, "MSG=HI91, ONTIME") == "2");
}

void configure(Serial & serial, const std::string & driver, const rclcpp::Logger & logger)
{
  if (driver == "yesense") {
    const bool baud_ok = yes_request(serial, 2) == Bytes{5};
    const bool rate_ok = yes_request(serial, 3) == Bytes{9};
    if (baud_ok && rate_ok) {return;}
    RCLCPP_INFO(logger, "configuring Yesense: 921600 baud / 200 Hz (save changed parameters)");
    if (!baud_ok) {
      // The baud-change ACK may be sent at either speed; verify by reading at the new speed.
      serial.clear();
      serial.write_all(yes_command(2, 2, {5}));
      pause_ms(100);
      serial.set_baud(921600);
      require(yes_request(serial, 2) == Bytes{5}, "Yesense baud readback mismatch");
    }
    if (!rate_ok) {
      require(yes_request(serial, 3, 2, {9}) == Bytes{0}, "Yesense rejected 200 Hz setting");
    }
    require(
      yes_request(serial, 2) == Bytes{5} && yes_request(serial, 3) == Bytes{9},
      "Yesense configuration readback mismatch");
  } else {
    const auto config = hip_request(serial, "LOG COMCONFIG");
    if (hip_target(config)) {
      hip_request(serial, "LOG ENABLE");
      return;
    }
    RCLCPP_INFO(logger, "configuring HiPNUC: 921600 baud / HI91 500 Hz (save and reboot)");
    try {
      hip_request(serial, "LOG DISABLE");
      if (field(config, "BAUD") != "921600") {
        hip_request(serial, "SERIALCONFIG 921600", true, 100);
        pause_ms(100);
        serial.set_baud(921600);
        hip_request(serial, "LOG DISABLE");
      }
      hip_request(serial, "LOG HI91 ONTIME 0.002");
      hip_request(serial, "SAVECONFIG");
      hip_request(serial, "REBOOT", true, 100);
      pause_ms(1000);
      hip_request(serial, "LOG DISABLE");
      const bool verified = hip_target(hip_request(serial, "LOG COMCONFIG"));
      hip_request(serial, "LOG ENABLE");
      require(verified, "HiPNUC configuration readback mismatch after reboot");
    } catch (...) {
      try {
        hip_request(serial, "LOG ENABLE");
      } catch (...) {
      }
      throw;
    }
  }
}
}  // namespace

std::string prepare_imu_device(
  const std::string & port, const std::string & driver, const rclcpp::Logger & logger)
{
  Serial serial(port);
  std::string detected, model;
  int detected_baud = 0;
  serial.set_baud(921600);
  // Native CDC firmware needs to settle once after open, not after every baud change.
  const auto tty = std::filesystem::canonical(port).filename().string();
  if (tty.rfind("ttyACM", 0) == 0) {pause_ms(150);}
  for (const int timeout_ms : {100, 500}) {
    for (const int baud : {921600, 115200, 460800}) {
      serial.set_baud(baud);
      RCLCPP_INFO(
        logger, "identifying IMU by parameter protocol: port=%s baud=%d timeout=%dms",
        port.c_str(), baud, timeout_ms);
      if (driver == "auto" || driver == "hipnuc") {
        try {
          serial.write_all({'\r', '\n'});
          pause_ms(20);
          model = field(hip_request(serial, "LOG VERSION", false, timeout_ms), "PNAME");
          if (!model.empty()) {detected = "hipnuc";}
        } catch (const std::exception &) {
        }
      }
      if (detected.empty() && (driver == "auto" || driver == "yesense")) {
        for (const int subtype : {2, 4}) {
          try {
            const auto reply = yes_request(serial, 0, 0, {uint8_t(subtype)}, timeout_ms);
            if (reply.size() < 2 || reply[0] != subtype || reply[1] == 0 ||
              reply[1] > reply.size() - 2) {continue;}
            model.clear();
            for (size_t i = 2; i < size_t(reply[1]) + 2 && reply[i]; ++i) {
              require(reply[i] >= 32 && reply[i] <= 126, "invalid Yesense product name");
              model += char(reply[i]);
            }
            if (!model.empty()) {detected = "yesense"; break;}
          } catch (const std::exception &) {
          }
        }
      }
      if (!detected.empty()) {detected_baud = baud; break;}
    }
    if (!detected.empty()) {break;}
  }
  require(!detected.empty(), "no valid HiPNUC/Yesense identity reply at 921600/115200/460800");
  RCLCPP_INFO(
    logger, "identified IMU: driver=%s model=%s port=%s connection_baud=%d",
    detected.c_str(), model.c_str(), port.c_str(), detected_baud);
  configure(serial, detected, logger);
  RCLCPP_INFO(
    logger, "IMU parameters verified: driver=%s baud=921600 frequency=%d Hz",
    detected.c_str(), detected == "yesense" ? 200 : 500);
  return detected;
}
}  // namespace bxi_imu
