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

#include "bxi_imu/hipnuc_backend.hpp"

#include <cerrno>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <thread>
#include <unistd.h>
#include <utility>

#include <asm/termbits.h>

extern "C" {
#include "hipnuc_dec.h"
}

namespace bxi_imu
{
namespace
{
constexpr double kDegreesToRadians = 0.017453292519943295;
constexpr double kG = 9.8;
constexpr double kMicroTeslaToTesla = 1.0e-6;
constexpr std::size_t kBufferSize = 1024;
}

HipnucBackend::HipnucBackend(
  std::string port,
  int baudrate,
  const rclcpp::Logger & logger)
: port_(std::move(port)), baudrate_(baudrate), logger_(logger)
{
}

HipnucBackend::~HipnucBackend()
{
  close();
}

bool HipnucBackend::open()
{
  if (opened_) {
    return true;
  }
  fd_ = open_serial_port();
  if (fd_ < 0) {
    return false;
  }
  opened_ = true;
  RCLCPP_INFO(logger_, "opened %s IMU on %s at %d baud", name().c_str(), port_.c_str(), baudrate_);
  return true;
}

void HipnucBackend::close()
{
  opened_ = false;
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool HipnucBackend::read(ImuSample & sample)
{
  if (!opened_ || fd_ < 0) {
    return false;
  }

  struct pollfd descriptor {};
  descriptor.fd = fd_;
  descriptor.events = POLLIN;
  const int poll_result = ::poll(&descriptor, 1, 100);
  if (poll_result == 0) {
    return false;
  }
  if (poll_result < 0) {
    if (errno != EINTR) {
      RCLCPP_ERROR(logger_, "poll(%s) failed: %s", port_.c_str(), std::strerror(errno));
    }
    return false;
  }
  if ((descriptor.revents & POLLIN) == 0) {
    if (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) {
      RCLCPP_ERROR(logger_, "serial port %s is no longer readable", port_.c_str());
      opened_ = false;
    }
    return false;
  }

  uint8_t buffer[kBufferSize]{};
  const ssize_t count = ::read(fd_, buffer, sizeof(buffer));
  if (count < 0) {
    if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
      RCLCPP_ERROR(logger_, "read(%s) failed: %s", port_.c_str(), std::strerror(errno));
    }
    return false;
  }
  if (count == 0) {
    return false;
  }

  for (ssize_t index = 0; index < count; ++index) {
    if (decode_byte(buffer[index], sample)) {
      return true;
    }
  }
  return false;
}

bool HipnucBackend::decode_byte(uint8_t byte, ImuSample & sample)
{
  static thread_local hipnuc_raw_t raw{};
  const int result = hipnuc_input(&raw, byte);
  // The decoder returns 1 only after a complete packet passes CRC and length checks.
  if (result <= 0) {
    return false;
  }

  sample = ImuSample{};
  if (raw.hi91.tag == 0x91) {
    sample.imu.orientation.w = raw.hi91.quat[0];
    sample.imu.orientation.x = raw.hi91.quat[1];
    sample.imu.orientation.y = raw.hi91.quat[2];
    sample.imu.orientation.z = raw.hi91.quat[3];
    sample.imu.angular_velocity.x = raw.hi91.gyr[0] * kDegreesToRadians;
    sample.imu.angular_velocity.y = raw.hi91.gyr[1] * kDegreesToRadians;
    sample.imu.angular_velocity.z = raw.hi91.gyr[2] * kDegreesToRadians;
    sample.imu.linear_acceleration.x = raw.hi91.acc[0] * kG;
    sample.imu.linear_acceleration.y = raw.hi91.acc[1] * kG;
    sample.imu.linear_acceleration.z = raw.hi91.acc[2] * kG;
    sample.magnetic.magnetic_field.x = raw.hi91.mag[0] * kMicroTeslaToTesla;
    sample.magnetic.magnetic_field.y = raw.hi91.mag[1] * kMicroTeslaToTesla;
    sample.magnetic.magnetic_field.z = raw.hi91.mag[2] * kMicroTeslaToTesla;
    sample.euler.vector.x = raw.hi91.roll * kDegreesToRadians;
    sample.euler.vector.y = raw.hi91.pitch * kDegreesToRadians;
    sample.euler.vector.z = raw.hi91.yaw * kDegreesToRadians;
    sample.temperature.temperature = raw.hi91.temp;
    sample.pressure.fluid_pressure = raw.hi91.air_pressure;
    sample.has_euler = true;
    sample.has_magnetic = true;
    sample.has_temperature = true;
    sample.has_pressure = true;
    set_now(sample);
    return true;
  }

  if (raw.hi83.tag == 0x83) {
    const uint32_t bitmap = raw.hi83.data_bitmap;
    if (bitmap & HI83_BMAP_QUAT) {
      sample.imu.orientation.w = raw.hi83.quat[0];
      sample.imu.orientation.x = raw.hi83.quat[1];
      sample.imu.orientation.y = raw.hi83.quat[2];
      sample.imu.orientation.z = raw.hi83.quat[3];
    }
    if (bitmap & HI83_BMAP_GYR_B) {
      sample.imu.angular_velocity.x = raw.hi83.gyr_b[0];
      sample.imu.angular_velocity.y = raw.hi83.gyr_b[1];
      sample.imu.angular_velocity.z = raw.hi83.gyr_b[2];
    }
    if (bitmap & HI83_BMAP_ACC_B) {
      sample.imu.linear_acceleration.x = raw.hi83.acc_b[0];
      sample.imu.linear_acceleration.y = raw.hi83.acc_b[1];
      sample.imu.linear_acceleration.z = raw.hi83.acc_b[2];
    }
    if (bitmap & HI83_BMAP_MAG_B) {
      sample.magnetic.magnetic_field.x = raw.hi83.mag_b[0] * kMicroTeslaToTesla;
      sample.magnetic.magnetic_field.y = raw.hi83.mag_b[1] * kMicroTeslaToTesla;
      sample.magnetic.magnetic_field.z = raw.hi83.mag_b[2] * kMicroTeslaToTesla;
      sample.has_magnetic = true;
    }
    if (bitmap & HI83_BMAP_RPY) {
      sample.euler.vector.x = raw.hi83.rpy[0] * kDegreesToRadians;
      sample.euler.vector.y = raw.hi83.rpy[1] * kDegreesToRadians;
      sample.euler.vector.z = raw.hi83.rpy[2] * kDegreesToRadians;
      sample.has_euler = true;
    }
    if (bitmap & HI83_BMAP_AIR_PRESSURE) {
      sample.pressure.fluid_pressure = raw.hi83.air_pressure;
      sample.has_pressure = true;
    }
    if (bitmap & HI83_BMAP_TEMPERATURE) {
      sample.temperature.temperature = raw.hi83.temperature;
      sample.has_temperature = true;
    }
    set_now(sample);
    return true;
  }
  return false;
}

void HipnucBackend::set_now(ImuSample & sample) const
{
  const auto stamp = rclcpp::Clock(RCL_SYSTEM_TIME).now();
  sample.imu.header.stamp = stamp;
  sample.euler.header.stamp = stamp;
  sample.magnetic.header.stamp = stamp;
  sample.temperature.header.stamp = stamp;
  sample.pressure.header.stamp = stamp;
}

int HipnucBackend::open_serial_port()
{
  const int serial = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (serial < 0) {
    RCLCPP_ERROR(logger_, "cannot open %s: %s", port_.c_str(), std::strerror(errno));
    return -1;
  }

  struct termios2 settings {};
  if (ioctl(serial, TCGETS2, &settings) != 0) {
    RCLCPP_ERROR(logger_, "TCGETS2(%s) failed: %s", port_.c_str(), std::strerror(errno));
    ::close(serial);
    return -1;
  }
  settings.c_cflag &= ~CBAUD;
  settings.c_cflag |= BOTHER | CS8;
  settings.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
  settings.c_ispeed = baudrate_;
  settings.c_ospeed = baudrate_;
  settings.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHONL | ISIG);
  settings.c_iflag &=
    ~(IXON | IXOFF | IXANY | IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
  settings.c_cc[VTIME] = 1;
  settings.c_cc[VMIN] = 0;

  if (ioctl(serial, TCSETS2, &settings) != 0) {
    RCLCPP_ERROR(logger_, "TCSETS2(%s) failed: %s", port_.c_str(), std::strerror(errno));
    ::close(serial);
    return -1;
  }
  return serial;
}

}  // namespace bxi_imu
