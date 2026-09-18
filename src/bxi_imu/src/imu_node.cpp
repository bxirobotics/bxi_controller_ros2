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
#include <cmath>
#include <cstdint>
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
    port_ = declare_parameter<std::string>("port", "/dev/ttyIMU");
    baudrate_ = declare_parameter<int>("baudrate", 921600);
    frame_id_ = declare_parameter<std::string>("frame_id", "imu_link");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/hardware/imu_data");
    euler_topic_ = declare_parameter<std::string>("euler_topic", "/euler_data");
    magnetic_topic_ = declare_parameter<std::string>("magnetic_topic", "/magnetic_data");
    temperature_topic_ = declare_parameter<std::string>("temperature_topic", "/temp_data");
    pressure_topic_ = declare_parameter<std::string>("pressure_topic", "/pressure_data");
    imu_enabled_ = declare_parameter<bool>("imu_enabled", true);
    quaternion_norm_tolerance_ = declare_parameter<double>(
      "quaternion_norm_tolerance", 0.1);
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

    if (!std::isfinite(quaternion_norm_tolerance_) ||
      quaternion_norm_tolerance_ < 0.0 || quaternion_norm_tolerance_ >= 1.0)
    {
      RCLCPP_WARN(
        get_logger(),
        "invalid quaternion_norm_tolerance=%.6f; using 0.1",
        quaternion_norm_tolerance_);
      quaternion_norm_tolerance_ = 0.1;
    }

    backend_ = create_backend(driver_, port_, baudrate_, get_logger());
    if (!backend_ || !backend_->open()) {
      RCLCPP_ERROR(
        get_logger(), "IMU node did not start: driver=%s port=%s",
        driver_.c_str(), port_.c_str());
      return;
    }

    RCLCPP_INFO(
      get_logger(),
      "using IMU backend '%s', publishing %s; quaternion norm validation "
      "enabled with tolerance %.3f (accepted range %.3f..%.3f)",
      backend_->name().c_str(), imu_topic_.c_str(), quaternion_norm_tolerance_,
      1.0 - quaternion_norm_tolerance_, 1.0 + quaternion_norm_tolerance_);
    running_ = true;
    startup_ok_ = true;
    reader_thread_ = std::thread([this]() {read_loop();});
  }

  bool startup_ok() const {return startup_ok_;}
  bool runtime_failed() const {return runtime_failed_.load();}

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
        if (!running_ || !rclcpp::ok()) {
          return;
        }
        if (!backend_->is_open()) {
          runtime_failed_ = true;
          running_ = false;
          RCLCPP_ERROR(
            get_logger(), "IMU backend '%s' lost access to %s; stopping IMU node",
            backend_->name().c_str(), port_.c_str());
          rclcpp::shutdown();
          return;
        }
        continue;
      }
      if (!valid_quaternion(sample.imu.orientation)) {
        ++invalid_quaternion_count_;
        const std::string dropped_count = std::to_string(invalid_quaternion_count_);
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "dropping IMU frame with invalid quaternion: "
          "w=%.6f x=%.6f y=%.6f z=%.6f norm=%.6f "
          "(accepted range %.3f..%.3f), dropped=%s",
          sample.imu.orientation.w, sample.imu.orientation.x,
          sample.imu.orientation.y, sample.imu.orientation.z,
          quaternion_norm(sample.imu.orientation),
          1.0 - quaternion_norm_tolerance_, 1.0 + quaternion_norm_tolerance_,
          dropped_count.c_str());
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
      if (!first_sample_logged_) {
        first_sample_logged_ = true;
        RCLCPP_INFO(
          get_logger(), "received first valid IMU frame from %s",
          port_.c_str());
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

  double quaternion_norm(const geometry_msgs::msg::Quaternion & quaternion) const
  {
    return std::sqrt(
      quaternion.w * quaternion.w + quaternion.x * quaternion.x +
      quaternion.y * quaternion.y + quaternion.z * quaternion.z);
  }

  bool valid_quaternion(const geometry_msgs::msg::Quaternion & quaternion) const
  {
    if (!std::isfinite(quaternion.w) || !std::isfinite(quaternion.x) ||
      !std::isfinite(quaternion.y) || !std::isfinite(quaternion.z))
    {
      return false;
    }

    const double norm = quaternion_norm(quaternion);
    return norm >= 1.0 - quaternion_norm_tolerance_ &&
           norm <= 1.0 + quaternion_norm_tolerance_;
  }

private:
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
  double quaternion_norm_tolerance_{0.1};
  bool euler_enabled_{false};
  bool magnetic_enabled_{false};
  bool temperature_enabled_{false};
  bool pressure_enabled_{false};
  std::uint64_t invalid_quaternion_count_{0};
  bool startup_ok_{false};
  bool first_sample_logged_{false};
  std::atomic<bool> runtime_failed_{false};

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
  auto node = std::make_shared<bxi_imu::ImuNode>(rclcpp::NodeOptions{});
  if (!node->startup_ok()) {
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::spin(node);
  const int exit_code = node->runtime_failed() ? 1 : 0;
  rclcpp::shutdown();
  return exit_code;
}
