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

#include <atomic>
#include <cstdint>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "bxi_imu/imu_backend.hpp"

namespace bxi_imu
{

class HipnucBackend final : public ImuBackend
{
public:
  HipnucBackend(
    std::string port,
    int baudrate,
    const rclcpp::Logger & logger);
  ~HipnucBackend() override;

  bool open() override;
  bool read(ImuSample & sample) override;
  void close() override;
  std::string name() const override {return "hipnuc";}

private:
  int open_serial_port();
  bool decode_byte(uint8_t byte, ImuSample & sample);
  void set_now(ImuSample & sample) const;

  std::string port_;
  int baudrate_;
  rclcpp::Logger logger_;
  int fd_{-1};
  std::atomic<bool> opened_{false};
};

}  // namespace bxi_imu
