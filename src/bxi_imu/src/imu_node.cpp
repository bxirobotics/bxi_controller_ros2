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
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <chrono>
#include <vector>
#include <regex>

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
    axis_mapping_ = declare_parameter<std::string>("axis_mapping", "y,-x,z");
    imu_frequency_hz_ = declare_parameter<double>("imu_frequency_hz", 200.0);
    imu_timeout_tolerance_ms_ = declare_parameter<double>("imu_timeout_tolerance_ms", 1.0);
    imu_candidates_ = declare_parameter<std::vector<std::string>>(
      "imu_candidates",
      std::vector<std::string>{
        "hipnuc,/dev/ttyIMU,921600",
        "yesense,/dev/ttyIMU_YESENSE_1,921600"});

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
    if (!std::isfinite(imu_frequency_hz_) || imu_frequency_hz_ <= 0.0) {
      RCLCPP_WARN(get_logger(), "invalid imu_frequency_hz; using 200 Hz");
      imu_frequency_hz_ = 200.0;
    }
    if (!std::isfinite(imu_timeout_tolerance_ms_) || imu_timeout_tolerance_ms_ < 0.0) {
      RCLCPP_WARN(get_logger(), "invalid imu_timeout_tolerance_ms; using 1 ms");
      imu_timeout_tolerance_ms_ = 1.0;
    }

    if (driver_ == "auto" || port_ == "auto") {
      select_backend_from_candidates();
    } else {
      backend_ = create_backend(driver_, port_, baudrate_, get_logger());
      if (backend_ && !backend_->open()) {
        backend_.reset();
      }
    }
    if (!backend_) {
      RCLCPP_ERROR(
        get_logger(), "IMU node did not start: driver=%s port=%s",
        driver_.c_str(), port_.c_str());
      return;
    }

    RCLCPP_INFO(
      get_logger(),
      "\n========== ACTIVE IMU ==========\n"
      "driver       : %s\n"
      "port         : %s\n"
      "baudrate     : %d\n"
      "imu_topic    : %s\n"
      "frame_id     : %s\n"
      "axis_mapping : %s\n"
      "frequency    : %.1f Hz (period %.3f ms, timeout %.3f ms)\n"
      "quat check   : enabled, norm %.3f..%.3f\n"
      "================================",
      backend_->name().c_str(), port_.c_str(), baudrate_, imu_topic_.c_str(),
      frame_id_.c_str(), axis_mapping_.c_str(), imu_frequency_hz_,
      expected_period_ms(), timeout_period_ms(),
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
  static int port_priority(const std::string & port)
  {
    static const std::regex suffix("_([0-9]+)$");
    std::smatch match;
    if (!std::regex_search(port, match, suffix)) {
      return 0;
    }
    try {
      return std::stoi(match[1].str()) + 1;
    } catch (const std::exception &) {
      return 0;
    }
  }

  void select_backend_from_candidates()
  {
    std::vector<std::string> candidates = imu_candidates_;
    std::stable_sort(candidates.begin(), candidates.end(), [](const std::string & left,
      const std::string & right) {
      const auto port_from_entry = [](const std::string & entry) {
        const auto first = entry.find(',');
        const auto second = entry.find(',', first == std::string::npos ? first : first + 1);
        return first == std::string::npos ? std::string{} :
          entry.substr(first + 1, second == std::string::npos ? std::string::npos : second - first - 1);
      };
      return port_priority(port_from_entry(left)) < port_priority(port_from_entry(right));
    });

    for (const auto & entry : candidates) {
      std::stringstream fields(entry);
      std::string candidate_driver;
      std::string candidate_port;
      std::string candidate_baudrate;
      if (!std::getline(fields, candidate_driver, ',') ||
        !std::getline(fields, candidate_port, ',') ||
        !std::getline(fields, candidate_baudrate, ','))
      {
        RCLCPP_WARN(get_logger(), "ignoring malformed imu_candidates entry '%s'", entry.c_str());
        continue;
      }

      try {
        const int baudrate = std::stoi(candidate_baudrate);
        auto candidate = create_backend(candidate_driver, candidate_port, baudrate, get_logger());
        if (candidate && candidate->open()) {
          driver_ = candidate_driver;
          port_ = candidate_port;
          baudrate_ = baudrate;
          backend_ = std::move(candidate);
          RCLCPP_INFO(
            get_logger(), "selected IMU candidate driver=%s port=%s baudrate=%d",
            driver_.c_str(), port_.c_str(), baudrate_);
          return;
        }
      } catch (const std::exception & error) {
        RCLCPP_WARN(
          get_logger(), "ignoring imu_candidates entry '%s': %s", entry.c_str(), error.what());
      }
    }
    RCLCPP_ERROR(get_logger(), "no usable IMU candidate found in imu_candidates");
  }

  void read_loop()
  {
    while (rclcpp::ok() && running_) {
      ImuSample sample;
      if (!backend_->read(sample)) {
        check_imu_timeout(false);
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
      check_imu_timeout(true);
      transform_sample_to_robot_frame(sample);
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

  void transform_sample_to_robot_frame(ImuSample & sample) const
  {
    if (axis_mapping_ == "y,-x,z") {
      const auto rotate = [](double x, double y, double z) {
        return std::array<double, 3>{y, -x, z};
      };

      const auto acceleration = rotate(
        sample.imu.linear_acceleration.x,
        sample.imu.linear_acceleration.y,
        sample.imu.linear_acceleration.z);
      sample.imu.linear_acceleration.x = acceleration[0];
      sample.imu.linear_acceleration.y = acceleration[1];
      sample.imu.linear_acceleration.z = acceleration[2];

      const auto angular_velocity = rotate(
        sample.imu.angular_velocity.x,
        sample.imu.angular_velocity.y,
        sample.imu.angular_velocity.z);
      sample.imu.angular_velocity.x = angular_velocity[0];
      sample.imu.angular_velocity.y = angular_velocity[1];
      sample.imu.angular_velocity.z = angular_velocity[2];

      const auto magnetic = rotate(
        sample.magnetic.magnetic_field.x,
        sample.magnetic.magnetic_field.y,
        sample.magnetic.magnetic_field.z);
      sample.magnetic.magnetic_field.x = magnetic[0];
      sample.magnetic.magnetic_field.y = magnetic[1];
      sample.magnetic.magnetic_field.z = magnetic[2];

      // q_robot = q_imu * q_(imu->robot), a +90 degree Z rotation here.
      constexpr double kHalfSqrtTwo = 0.70710678118654752440;
      const auto input = sample.imu.orientation;
      sample.imu.orientation.w = (input.w - input.z) * kHalfSqrtTwo;
      sample.imu.orientation.x = (input.x + input.y) * kHalfSqrtTwo;
      sample.imu.orientation.y = (-input.x + input.y) * kHalfSqrtTwo;
      sample.imu.orientation.z = (input.w + input.z) * kHalfSqrtTwo;
      return;
    }

    if (axis_mapping_ != "identity") {
      RCLCPP_WARN_ONCE(
        get_logger(),
        "unsupported axis_mapping='%s'; using identity mapping",
        axis_mapping_.c_str());
    }
  }

  double expected_period_ms() const
  {
    return 1000.0 / imu_frequency_hz_;
  }

  double timeout_period_ms() const
  {
    return expected_period_ms() + imu_timeout_tolerance_ms_;
  }

  void check_imu_timeout(bool sample_received)
  {
    if (imu_frequency_hz_ <= 0.0 || imu_timeout_tolerance_ms_ < 0.0) {
      return;
    }

    const auto now = std::chrono::steady_clock::now();
    if (sample_received) {
      if (last_sample_time_.has_value()) {
        const double gap_ms = std::chrono::duration<double, std::milli>(
          now - *last_sample_time_).count();
        if (gap_ms > timeout_period_ms()) {
          if (!timeout_reported_) {
            RCLCPP_WARN(
              get_logger(),
              "IMU frame timeout: gap=%.3f ms, expected=%.3f ms, threshold=%.3f ms",
              gap_ms, expected_period_ms(), timeout_period_ms());
          }
          timeout_reported_ = true;
        } else if (timeout_reported_) {
          RCLCPP_INFO(
            get_logger(), "IMU data recovered: gap=%.3f ms", gap_ms);
        }
      }
      last_sample_time_ = now;
      timeout_reported_ = false;
      return;
    }

    if (last_sample_time_.has_value() && !timeout_reported_) {
      const double gap_ms = std::chrono::duration<double, std::milli>(
        now - *last_sample_time_).count();
      if (gap_ms > timeout_period_ms()) {
        RCLCPP_WARN(
          get_logger(),
          "IMU data timeout: no valid frame for %.3f ms, expected period=%.3f ms",
          gap_ms, expected_period_ms());
        timeout_reported_ = true;
      }
    }
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
  std::string axis_mapping_;
  double imu_frequency_hz_{200.0};
  double imu_timeout_tolerance_ms_{1.0};
  bool imu_enabled_{true};
  double quaternion_norm_tolerance_{0.1};
  bool euler_enabled_{false};
  bool magnetic_enabled_{false};
  bool temperature_enabled_{false};
  bool pressure_enabled_{false};
  std::vector<std::string> imu_candidates_;
  std::uint64_t invalid_quaternion_count_{0};
  bool startup_ok_{false};
  bool first_sample_logged_{false};
  std::atomic<bool> runtime_failed_{false};
  std::optional<std::chrono::steady_clock::time_point> last_sample_time_;
  bool timeout_reported_{false};

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
