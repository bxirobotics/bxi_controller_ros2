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

#include <atomic>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "bxi_imu/imu_backend.hpp"

namespace bxi_imu
{

class ImuNode final : public rclcpp::Node
{
public:
  explicit ImuNode(const rclcpp::NodeOptions & options)
  : Node("imu_node", options)
  {
    driver_ = declare_parameter<std::string>("driver", "hipnuc");
    port_ = declare_parameter<std::string>("port", "/dev/ttyIMU_2");
    baudrate_ = declare_parameter<int>("baudrate", 921600);
    frame_id_ = declare_parameter<std::string>("frame_id", "imu_link");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/hardware/imu_data");
    euler_topic_ = declare_parameter<std::string>("euler_topic", "/euler_data");
    magnetic_topic_ = declare_parameter<std::string>("magnetic_topic", "/magnetic_data");
    temperature_topic_ = declare_parameter<std::string>("temperature_topic", "/temp_data");
    pressure_topic_ = declare_parameter<std::string>("pressure_topic", "/pressure_data");
    imu_enabled_ = declare_parameter<bool>("imu_enabled", true);
    euler_enabled_ = declare_parameter<bool>("euler_enabled", false);
    magnetic_enabled_ = declare_parameter<bool>("magnetic_enabled", false);
    temperature_enabled_ = declare_parameter<bool>("temperature_enabled", false);
    pressure_enabled_ = declare_parameter<bool>("pressure_enabled", false);

    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic_, rclcpp::SensorDataQoS());
    euler_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
      euler_topic_,
      rclcpp::SensorDataQoS());
    magnetic_pub_ = create_publisher<sensor_msgs::msg::MagneticField>(
      magnetic_topic_,
      rclcpp::SensorDataQoS());
    temperature_pub_ = create_publisher<sensor_msgs::msg::Temperature>(
      temperature_topic_,
      rclcpp::SensorDataQoS());
    pressure_pub_ = create_publisher<sensor_msgs::msg::FluidPressure>(
      pressure_topic_,
      rclcpp::SensorDataQoS());

    backend_ = create_backend(driver_, port_, baudrate_, get_logger());
    if (!backend_ || !backend_->open()) {
      RCLCPP_ERROR(
        get_logger(), "IMU node did not start: driver=%s port=%s",
        driver_.c_str(), port_.c_str());
      return;
    }

    RCLCPP_INFO(
      get_logger(), "using IMU backend '%s', publishing %s",
      backend_->name().c_str(), imu_topic_.c_str());
    running_ = true;
    reader_thread_ = std::thread([this]() {read_loop();});
  }

  ~ImuNode() override
  {
    running_ = false;
    if (backend_) {
      backend_->close();
    }
    if (reader_thread_.joinable()) {
      reader_thread_.join();
    }
  }

private:
  void read_loop()
  {
    while (rclcpp::ok() && running_) {
      ImuSample sample;
      if (!backend_->read(sample)) {
        continue;
      }
      stamp_and_frame(sample);
      if (imu_enabled_) {
        imu_pub_->publish(sample.imu);
      }
      if (euler_enabled_ && sample.has_euler) {
        euler_pub_->publish(sample.euler);
      }
      if (magnetic_enabled_ && sample.has_magnetic) {
        magnetic_pub_->publish(sample.magnetic);
      }
      if (temperature_enabled_ && sample.has_temperature) {
        temperature_pub_->publish(sample.temperature);
      }
      if (pressure_enabled_ && sample.has_pressure) {
        pressure_pub_->publish(sample.pressure);
      }
    }
  }

  void stamp_and_frame(ImuSample & sample) const
  {
    sample.imu.header.frame_id = frame_id_;
    sample.euler.header.frame_id = frame_id_;
    sample.magnetic.header.frame_id = frame_id_;
    sample.temperature.header.frame_id = frame_id_;
    sample.pressure.header.frame_id = frame_id_;
  }

  std::string driver_;
  std::string port_;
  int baudrate_{0};
  std::string frame_id_;
  std::string imu_topic_;
  std::string euler_topic_;
  std::string magnetic_topic_;
  std::string temperature_topic_;
  std::string pressure_topic_;
  bool imu_enabled_{true};
  bool euler_enabled_{false};
  bool magnetic_enabled_{false};
  bool temperature_enabled_{false};
  bool pressure_enabled_{false};

  BackendPtr backend_;
  std::atomic<bool> running_{false};
  std::thread reader_thread_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr euler_pub_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr magnetic_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperature_pub_;
  rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr pressure_pub_;
};

}  // namespace bxi_imu

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<bxi_imu::ImuNode>(rclcpp::NodeOptions{}));
  rclcpp::shutdown();
  return 0;
}
