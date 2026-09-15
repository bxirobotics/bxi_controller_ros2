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

namespace bxi_imu
{

BackendPtr create_backend(
  const std::string & driver,
  const std::string & port,
  int baudrate,
  const rclcpp::Logger & logger)
{
  if (driver == "hipnuc") {
    return std::make_unique<HipnucBackend>(port, baudrate, logger);
  }

  RCLCPP_ERROR(
    logger,
    "unsupported IMU driver '%s'. Add a backend and register it in src/backend_factory.cpp",
    driver.c_str());
  return nullptr;
}

}  // namespace bxi_imu
