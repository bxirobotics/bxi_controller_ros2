// Copyright 2026 BXI Robotics

#include "yesense_backend.hpp"

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <utility>

#include <asm/termbits.h>

#include "lib/yesense_decoder_comm.h"

namespace bxi_imu
{
namespace
{
constexpr double kDegreesToRadians = 0.017453292519943295;
constexpr double kMicroTeslaToTesla = 1.0e-6;
constexpr std::size_t kBufferSize = 4096;
constexpr std::size_t kDecodeChunkSize = 512;
constexpr std::size_t kMaxDrainBytes = 64 * 1024;
}

YesenseBackend::YesenseBackend(
  std::string port, int baudrate, const rclcpp::Logger & logger)
: port_(std::move(port)), baudrate_(baudrate), logger_(logger)
{
  std::memset(&decoded_, 0, sizeof(decoded_));
}

YesenseBackend::~YesenseBackend()
{
  close();
}

bool YesenseBackend::open()
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

void YesenseBackend::close()
{
  opened_ = false;
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool YesenseBackend::read(ImuSample & sample)
{
  if (!opened_ || fd_ < 0) {
    return false;
  }

  struct pollfd descriptor{};
  descriptor.fd = fd_;
  descriptor.events = POLLIN;
  const int poll_result = ::poll(&descriptor, 1, 100);
  if (poll_result == 0) {
    return false;
  }
  if (poll_result < 0) {
    if (errno != EINTR && opened_) {
      RCLCPP_ERROR(logger_, "poll(%s) failed: %s", port_.c_str(), std::strerror(errno));
      opened_ = false;
    }
    return false;
  }
  if (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) {
    RCLCPP_ERROR(logger_, "serial port %s is no longer readable", port_.c_str());
    opened_ = false;
    return false;
  }

  bool found = false;
  std::size_t drained_bytes = 0;
  for (;;) {
    uint8_t buffer[kBufferSize]{};
    const ssize_t count = ::read(fd_, buffer, sizeof(buffer));
    if (count < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        return found;
      }
      if (errno == EINTR) {
        continue;
      }
      RCLCPP_ERROR(logger_, "read(%s) failed: %s", port_.c_str(), std::strerror(errno));
      opened_ = false;
      return false;
    }
    if (count == 0) {
      RCLCPP_ERROR(logger_, "serial port %s returned EOF", port_.c_str());
      opened_ = false;
      return false;
    }

    drained_bytes += static_cast<std::size_t>(count);
    ImuSample latest;
    if (decode(buffer, static_cast<std::size_t>(count), latest)) {
      sample = std::move(latest);
      found = true;
    }
    if (drained_bytes >= kMaxDrainBytes) {
      int queued_bytes = 0;
      if (ioctl(fd_, FIONREAD, &queued_bytes) != 0 || queued_bytes > 0) {
        RCLCPP_ERROR(logger_,
          "Yesense serial backlog exceeded %zu bytes; discarding queued IMU data",
          kMaxDrainBytes);
        if (ioctl(fd_, TCIFLUSH, 0) != 0) {
          RCLCPP_ERROR(logger_, "cannot flush stale Yesense data: %s", std::strerror(errno));
          opened_ = false;
        }
        decoder_.clear_buffer();
        std::memset(&decoded_, 0, sizeof(decoded_));
        return false;
      }
      return found;
    }
  }
}

bool YesenseBackend::decode(const uint8_t * data, std::size_t length, ImuSample & sample)
{
  bool found = false;
  std::size_t complete_frames = 0;
  for (std::size_t offset = 0; offset < length; offset += kDecodeChunkSize) {
    const auto chunk_size = std::min(kDecodeChunkSize, length - offset);
    int result = decoder_.data_proc(
      const_cast<unsigned char *>(data + offset), static_cast<unsigned int>(chunk_size), &decoded_);
    while (result == analysis_ok || result == crc_err) {
      if (result == crc_err) {
        std::memset(&decoded_, 0, sizeof(decoded_));
      } else if (decoded_.content.valid_flg && decoded_.content.acc &&
        decoded_.content.gyro && decoded_.content.quat)
      {
        sample = ImuSample{};
        sample.imu.orientation.w = decoded_.quat.q0;
        sample.imu.orientation.x = decoded_.quat.q1;
        sample.imu.orientation.y = decoded_.quat.q2;
        sample.imu.orientation.z = decoded_.quat.q3;
        sample.imu.angular_velocity.x = decoded_.gyro.x * kDegreesToRadians;
        sample.imu.angular_velocity.y = decoded_.gyro.y * kDegreesToRadians;
        sample.imu.angular_velocity.z = decoded_.gyro.z * kDegreesToRadians;
        sample.imu.linear_acceleration.x = decoded_.acc.x;
        sample.imu.linear_acceleration.y = decoded_.acc.y;
        sample.imu.linear_acceleration.z = decoded_.acc.z;
        sample.magnetic.magnetic_field.x = decoded_.mag_norm.x * kMicroTeslaToTesla;
        sample.magnetic.magnetic_field.y = decoded_.mag_norm.y * kMicroTeslaToTesla;
        sample.magnetic.magnetic_field.z = decoded_.mag_norm.z * kMicroTeslaToTesla;
        sample.euler.vector.x = decoded_.euler.roll * kDegreesToRadians;
        sample.euler.vector.y = decoded_.euler.pitch * kDegreesToRadians;
        sample.euler.vector.z = decoded_.euler.yaw * kDegreesToRadians;
        sample.temperature.temperature = decoded_.sensor_temp;
        sample.pressure.fluid_pressure = decoded_.pressure;
        sample.has_euler = decoded_.content.euler != 0;
        sample.has_magnetic = decoded_.content.mag_norm != 0;
        sample.has_temperature = decoded_.content.sensor_temp != 0;
        sample.has_pressure = decoded_.content.pressure != 0;
        sample.device_frame_id = decoded_.tid;
        if (decoded_.content.sample_timestamp) {
          sample.device_tick = decoded_.sample_timestamp;
          sample.device_tick_modulus = std::uint64_t{1} << 32;
          sample.device_tick_kind = 3;
        }
        std::memset(&decoded_, 0, sizeof(decoded_));
        found = true;
        ++complete_frames;
      }
      decoded_.content.valid_flg = 0;
      result = decoder_.data_proc(
        const_cast<unsigned char *>(data + offset), 0u, &decoded_);
    }
  }
  if (found) {
    set_now(sample);
    if (complete_frames > 1) {
      static rclcpp::Clock steady_clock(RCL_STEADY_TIME);
      RCLCPP_WARN_THROTTLE(
        logger_, steady_clock, 5000,
        "coalesced %zu Yesense IMU frames in one serial read; using the newest",
        complete_frames);
    }
  }
  return found;
}

void YesenseBackend::set_now(ImuSample & sample) const
{
  const auto stamp = rclcpp::Clock(RCL_SYSTEM_TIME).now();
  sample.imu.header.stamp = stamp;
  sample.euler.header.stamp = stamp;
  sample.magnetic.header.stamp = stamp;
  sample.temperature.header.stamp = stamp;
  sample.pressure.header.stamp = stamp;
}

int YesenseBackend::open_serial_port()
{
  const int serial = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (serial < 0) {
    RCLCPP_WARN(logger_, "cannot open %s: %s", port_.c_str(), std::strerror(errno));
    return -1;
  }
  if (ioctl(serial, TIOCEXCL) != 0) {
    RCLCPP_WARN(logger_, "cannot exclusively claim %s: %s", port_.c_str(), std::strerror(errno));
    ::close(serial);
    return -1;
  }

  struct termios2 settings{};
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
  ioctl(serial, TCIOFLUSH, 0);
  return serial;
}

}  // namespace bxi_imu

extern "C" const char * bxi_imu_driver_name()
{
  return "yesense";
}

extern "C" bxi_imu::ImuBackend * bxi_imu_create_backend(
  const std::string & port, int baudrate, const rclcpp::Logger & logger)
{
  return new bxi_imu::YesenseBackend(port, baudrate, logger);
}
