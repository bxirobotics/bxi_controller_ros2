// Copyright 2026 BXI Robotics
#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "bxi_imu/imu_backend.hpp"
#include "lib/yesense_decoder.h"

namespace bxi_imu
{

class YesenseBackend final : public ImuBackend
{
public:
  YesenseBackend(std::string port, int baudrate, const rclcpp::Logger & logger);
  ~YesenseBackend() override;

  bool open() override;
  bool read(ImuSample & sample) override;
  bool is_open() const override {return opened_.load();}
  void close() override;
  std::string name() const override {return "yesense";}

private:
  bool decode(const uint8_t * data, std::size_t length, ImuSample & sample);
  int open_serial_port();
  void set_now(ImuSample & sample) const;

  std::string port_;
  int baudrate_;
  rclcpp::Logger logger_;
  int fd_{-1};
  std::atomic<bool> opened_{false};
  yesense::yesense_decoder decoder_;
  yesense::yis_out_data_t decoded_{};
};

}  // namespace bxi_imu
