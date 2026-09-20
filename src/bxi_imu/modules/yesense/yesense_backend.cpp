// Copyright 2026 BXI Robotics

#include "yesense_backend.hpp"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <utility>

#include <asm/termbits.h>

namespace bxi_imu
{
namespace
{
constexpr double kDegreesToRadians = 0.017453292519943295;
constexpr double kMicroTeslaToTesla = 1.0e-6;
constexpr std::size_t kBufferSize = 4096;
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

  uint8_t buffer[kBufferSize]{};
  const ssize_t count = ::read(fd_, buffer, sizeof(buffer));
  if (count < 0) {
    if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR && opened_) {
      RCLCPP_ERROR(logger_, "read(%s) failed: %s", port_.c_str(), std::strerror(errno));
      opened_ = false;
    }
    return false;
  }
  if (count == 0) {
    RCLCPP_ERROR(logger_, "serial port %s returned EOF", port_.c_str());
    opened_ = false;
    return false;
  }

  return decode(buffer, static_cast<std::size_t>(count), sample);
}

bool YesenseBackend::decode(const uint8_t * data, std::size_t length, ImuSample & sample)
{
  decoder_.data_proc(const_cast<unsigned char *>(data), static_cast<unsigned int>(length), &decoded_);
  if (!(decoded_.content.acc && decoded_.content.gyro && decoded_.content.quat)) {
    return false;
  }

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
  set_now(sample);
  std::memset(&decoded_, 0, sizeof(decoded_));
  return true;
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
