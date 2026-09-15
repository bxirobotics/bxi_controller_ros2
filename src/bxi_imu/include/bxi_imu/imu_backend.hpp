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

#include <memory>
#include <string>

#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/temperature.hpp>

namespace bxi_imu
{

struct ImuSample
{
  ImuSample()
  : imu(rosidl_runtime_cpp::MessageInitialization::ALL),
    euler(rosidl_runtime_cpp::MessageInitialization::ALL),
    magnetic(rosidl_runtime_cpp::MessageInitialization::ALL),
    temperature(rosidl_runtime_cpp::MessageInitialization::ALL),
    pressure(rosidl_runtime_cpp::MessageInitialization::ALL)
  {
  }

  sensor_msgs::msg::Imu imu;
  geometry_msgs::msg::Vector3Stamped euler;
  sensor_msgs::msg::MagneticField magnetic;
  sensor_msgs::msg::Temperature temperature;
  sensor_msgs::msg::FluidPressure pressure;
  bool has_euler{false};
  bool has_magnetic{false};
  bool has_temperature{false};
  bool has_pressure{false};
};

class ImuBackend
{
public:
  virtual ~ImuBackend() = default;

  virtual bool open() = 0;
  // Returns true for one fully decoded sample and false on timeout/error.
  virtual bool read(ImuSample & sample) = 0;
  virtual void close() = 0;
  virtual std::string name() const = 0;
};

using BackendPtr = std::unique_ptr<ImuBackend>;

BackendPtr create_backend(
  const std::string & driver,
  const std::string & port,
  int baudrate,
  const rclcpp::Logger & logger);

}  // namespace bxi_imu
