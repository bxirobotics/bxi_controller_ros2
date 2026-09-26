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
#include <condition_variable>
#include <cstdint>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <optional>
#include <deque>
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
    axis_mapping_ = declare_parameter<std::string>("axis_mapping", "identity");
    imu_frequency_hz_ = declare_parameter<double>("imu_frequency_hz", 200.0);
    imu_timeout_multiplier_ = declare_parameter<double>("imu_timeout_multiplier", 1.5);
    imu_record_enabled_ = declare_parameter<bool>("imu_record_enabled", false);
    imu_record_enabled_override_ = declare_parameter<std::string>(
      "imu_record_enabled_override", "auto");
    imu_record_dir_ = declare_parameter<std::string>(
      "imu_record_dir", "/var/log/bxi_log/imu/data");
    imu_record_max_files_ = declare_parameter<int>("imu_record_max_files", 10);
    imu_candidates_ = declare_parameter<std::vector<std::string>>(
      "imu_candidates",
      std::vector<std::string>{
        "hipnuc|/dev/ttyIMU|921600|identity|500.0|2.5",
        "yesense|/dev/ttyIMU_YESENSE_1|921600|-y,x,z|200.0|2.5"});

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
    if (!std::isfinite(imu_timeout_multiplier_) || imu_timeout_multiplier_ < 1.0) {
      RCLCPP_WARN(get_logger(), "invalid imu_timeout_multiplier; using 1.5");
      imu_timeout_multiplier_ = 1.5;
    }

    if (driver_ == "auto" || port_ == "auto") {
      select_backend_from_candidates();
    } else {
      for (const auto & entry : imu_candidates_) {
        CandidateConfig candidate_config;
        if (parse_candidate(entry, candidate_config) &&
          candidate_config.driver == driver_ && candidate_config.port == port_)
        {
          apply_candidate_config(candidate_config);
          break;
        }
      }
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
    apply_record_enabled_override();
    if (!std::isfinite(quaternion_norm_tolerance_) ||
      quaternion_norm_tolerance_ < 0.0 || quaternion_norm_tolerance_ >= 1.0)
    {
      RCLCPP_WARN(
        get_logger(),
        "invalid quaternion_norm_tolerance=%.6f; using 0.1",
        quaternion_norm_tolerance_);
      quaternion_norm_tolerance_ = 0.1;
    }
    sanitize_timing_parameters();
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

    RCLCPP_INFO(
      get_logger(),
      "\n========== ACTIVE IMU ==========\n"
      "driver       : %s\n"
      "port         : %s\n"
      "baudrate     : %d\n"
      "imu_topic    : %s\n"
      "frame_id     : %s\n"
      "axis_mapping : %s\n"
      "frequency    : %.1f Hz (period %.3f ms, timeout %.2fx / %.3f ms)\n"
      "quat check   : enabled, norm %.3f..%.3f\n"
      "================================",
      backend_->name().c_str(), port_.c_str(), baudrate_, imu_topic_.c_str(),
      frame_id_.c_str(), axis_mapping_.c_str(), imu_frequency_hz_,
      expected_period_ms(), imu_timeout_multiplier_, timeout_period_ms(),
      1.0 - quaternion_norm_tolerance_, 1.0 + quaternion_norm_tolerance_);
    open_imu_recorder();
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
    close_imu_recorder();
  }

private:
  struct CandidateConfig
  {
    std::string driver;
    std::string port;
    int baudrate{921600};
    std::string axis_mapping{"identity"};
    double frequency_hz{200.0};
    double timeout_multiplier{1.5};
    bool has_module_parameters{false};
    std::string frame_id;
    std::string imu_topic;
    std::string euler_topic;
    std::string magnetic_topic;
    std::string temperature_topic;
    std::string pressure_topic;
    bool imu_enabled{true};
    double quaternion_norm_tolerance{0.1};
    bool euler_enabled{false};
    bool magnetic_enabled{false};
    bool temperature_enabled{false};
    bool pressure_enabled{false};
    bool record_enabled{false};
    std::string record_dir;
    int record_max_files{10};
  };

  static bool parse_candidate(const std::string & entry, CandidateConfig & candidate)
  {
    std::stringstream fields(entry);
    std::string baudrate;
    std::string frequency;
    std::string timeout_multiplier;
    if (!std::getline(fields, candidate.driver, '|') ||
      !std::getline(fields, candidate.port, '|') ||
      !std::getline(fields, baudrate, '|') ||
      !std::getline(fields, candidate.axis_mapping, '|') ||
      !std::getline(fields, frequency, '|') ||
      !std::getline(fields, timeout_multiplier, '|'))
    {
      return false;
    }
    try {
      candidate.baudrate = std::stoi(baudrate);
      candidate.frequency_hz = std::stod(frequency);
      candidate.timeout_multiplier = std::stod(timeout_multiplier);
    } catch (const std::exception &) {
      return false;
    }
    if (candidate.driver.empty() || candidate.port.empty()) {
      return false;
    }

    std::vector<std::string> module_fields;
    std::string field;
    while (std::getline(fields, field, '|')) {
      module_fields.push_back(field);
    }
    if (module_fields.empty()) {
      return true;
    }
    if (module_fields.size() != 15) {
      return false;
    }
    try {
      candidate.frame_id = module_fields[0];
      candidate.imu_topic = module_fields[1];
      candidate.euler_topic = module_fields[2];
      candidate.magnetic_topic = module_fields[3];
      candidate.temperature_topic = module_fields[4];
      candidate.pressure_topic = module_fields[5];
      const auto parse_bool = [](const std::string & value, bool & output) {
          if (value == "true" || value == "1") {
            output = true;
            return true;
          }
          if (value == "false" || value == "0") {
            output = false;
            return true;
          }
          return false;
        };
      if (!parse_bool(module_fields[6], candidate.imu_enabled) ||
        !parse_bool(module_fields[8], candidate.euler_enabled) ||
        !parse_bool(module_fields[9], candidate.magnetic_enabled) ||
        !parse_bool(module_fields[10], candidate.temperature_enabled) ||
        !parse_bool(module_fields[11], candidate.pressure_enabled) ||
        !parse_bool(module_fields[12], candidate.record_enabled))
      {
        return false;
      }
      candidate.quaternion_norm_tolerance = std::stod(module_fields[7]);
      candidate.record_dir = module_fields[13];
      candidate.record_max_files = std::stoi(module_fields[14]);
      candidate.has_module_parameters = true;
    } catch (const std::exception &) {
      return false;
    }
    return true;
  }

  void apply_candidate_config(const CandidateConfig & candidate)
  {
    axis_mapping_ = candidate.axis_mapping;
    imu_frequency_hz_ = candidate.frequency_hz;
    imu_timeout_multiplier_ = candidate.timeout_multiplier;
    if (!candidate.has_module_parameters) {
      return;
    }
    frame_id_ = candidate.frame_id;
    imu_topic_ = candidate.imu_topic;
    euler_topic_ = candidate.euler_topic;
    magnetic_topic_ = candidate.magnetic_topic;
    temperature_topic_ = candidate.temperature_topic;
    pressure_topic_ = candidate.pressure_topic;
    imu_enabled_ = candidate.imu_enabled;
    quaternion_norm_tolerance_ = candidate.quaternion_norm_tolerance;
    euler_enabled_ = candidate.euler_enabled;
    magnetic_enabled_ = candidate.magnetic_enabled;
    temperature_enabled_ = candidate.temperature_enabled;
    pressure_enabled_ = candidate.pressure_enabled;
    imu_record_enabled_ = candidate.record_enabled;
    imu_record_dir_ = candidate.record_dir;
    imu_record_max_files_ = candidate.record_max_files;
  }

  void apply_record_enabled_override()
  {
    if (imu_record_enabled_override_ == "auto") {
      return;
    }
    if (imu_record_enabled_override_ == "true" || imu_record_enabled_override_ == "1") {
      imu_record_enabled_ = true;
      return;
    }
    if (imu_record_enabled_override_ == "false" || imu_record_enabled_override_ == "0") {
      imu_record_enabled_ = false;
      return;
    }
    RCLCPP_WARN(
      get_logger(), "invalid imu_record_enabled_override='%s'; using module setting",
      imu_record_enabled_override_.c_str());
  }

  void sanitize_timing_parameters()
  {
    if (!std::isfinite(imu_frequency_hz_) || imu_frequency_hz_ <= 0.0) {
      RCLCPP_WARN(get_logger(), "invalid imu_frequency_hz; using 200 Hz");
      imu_frequency_hz_ = 200.0;
    }
    if (!std::isfinite(imu_timeout_multiplier_) || imu_timeout_multiplier_ < 1.0) {
      RCLCPP_WARN(get_logger(), "invalid imu_timeout_multiplier; using 1.5");
      imu_timeout_multiplier_ = 1.5;
    }
  }

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
        CandidateConfig candidate;
        return parse_candidate(entry, candidate) ? candidate.port : std::string{};
      };
      return port_priority(port_from_entry(left)) < port_priority(port_from_entry(right));
    });

    for (const auto & entry : candidates) {
      CandidateConfig candidate_config;
      if (!parse_candidate(entry, candidate_config))
      {
        RCLCPP_WARN(get_logger(), "ignoring malformed imu_candidates entry '%s'", entry.c_str());
        continue;
      }

      try {
        auto candidate = create_backend(
          candidate_config.driver, candidate_config.port, candidate_config.baudrate, get_logger());
        if (candidate && candidate->open()) {
          driver_ = candidate_config.driver;
          port_ = candidate_config.port;
          baudrate_ = candidate_config.baudrate;
          apply_candidate_config(candidate_config);
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
      const bool quaternion_valid = valid_quaternion(sample.imu.orientation);
      record_sample(
        sample, quaternion_valid, quaternion_valid ? "" : "invalid_quaternion");
      if (!quaternion_valid) {
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

  static std::string local_timestamp()
  {
    const auto now = std::chrono::system_clock::now();
    const std::time_t time = std::chrono::system_clock::to_time_t(now);
    std::tm local_time{};
#if defined(_WIN32)
    localtime_s(&local_time, &time);
#else
    localtime_r(&time, &local_time);
#endif
    std::ostringstream output;
    output << std::put_time(&local_time, "%Y%m%d_%H%M%S");
    return output.str();
  }

  void open_imu_recorder()
  {
    if (!imu_record_enabled_) {
      return;
    }
    if (imu_record_max_files_ < 1) {
      RCLCPP_WARN(get_logger(), "invalid imu_record_max_files; recording disabled");
      return;
    }

    std::error_code error;
    std::filesystem::create_directories(imu_record_dir_, error);
    if (error) {
      RCLCPP_WARN(
        get_logger(), "cannot create IMU record directory %s: %s; recording disabled",
        imu_record_dir_.c_str(), error.message().c_str());
      return;
    }
    record_file_prefix_ = "imu_data_" + local_timestamp();
    rotate_imu_record_file();
    if (record_file_.is_open()) {
      RCLCPP_INFO(
      get_logger(),
        "IMU CSV recording enabled: dir=%s one_file_per_start keep=%d files",
        imu_record_dir_.c_str(), imu_record_max_files_);
    }
  }

  struct RecordedSample
  {
    ImuSample sample;
    double arrival_gap_ms{0.0};
    bool quaternion_valid{false};
    std::string drop_reason;
  };

  void rotate_imu_record_file()
  {
    if (record_file_.is_open()) {
      record_file_.flush();
      record_file_.close();
    }

    ++record_file_index_;
    std::ostringstream filename;
    filename << record_file_prefix_ << "_" << std::setfill('0') << std::setw(4)
             << record_file_index_ << ".csv";
    current_record_path_ = std::filesystem::path(imu_record_dir_) / filename.str();
    record_file_.open(current_record_path_, std::ios::out | std::ios::trunc);
    if (!record_file_.is_open()) {
      RCLCPP_WARN(
        get_logger(), "cannot open IMU record file %s; recording disabled",
        current_record_path_.c_str());
      imu_record_enabled_ = false;
      return;
    }
    record_file_ << "# driver=" << backend_->name() << "\n"
                 << "# port=" << port_ << "\n"
                 << "# axis_mapping=" << axis_mapping_ << "\n"
                 << "# frequency_hz=" << imu_frequency_hz_ << "\n"
                 << "receive_time_ns,ros_stamp_sec,ros_stamp_nanosec,"
                 << "quaternion_valid,drop_reason,"
                 << "quaternion_w,quaternion_x,quaternion_y,quaternion_z,quaternion_norm,"
                 << "euler_roll,euler_pitch,euler_yaw,"
                 << "angular_velocity_x,angular_velocity_y,angular_velocity_z,"
                 << "linear_acceleration_x,linear_acceleration_y,linear_acceleration_z,"
                 << "arrival_gap_ms\n";
    record_file_.flush();
    if (record_file_.fail()) {
      RCLCPP_WARN(get_logger(), "cannot write IMU record header; recording disabled");
      record_file_.close();
      imu_record_enabled_ = false;
      return;
    }
    recorded_rows_since_flush_ = 0;
    prune_imu_record_files();
    recording_running_ = true;
    record_writer_thread_ = std::thread([this]() {record_writer_loop();});
  }

  void prune_imu_record_files()
  {
    std::vector<std::filesystem::path> files;
    std::error_code error;
    for (const auto & entry : std::filesystem::directory_iterator(imu_record_dir_, error)) {
      if (error) {
        break;
      }
      if (entry.is_regular_file() && entry.path().filename().string().rfind("imu_data_", 0) == 0 &&
        entry.path().extension() == ".csv")
      {
        files.push_back(entry.path());
      }
    }
    std::sort(files.begin(), files.end());
    while (static_cast<int>(files.size()) > imu_record_max_files_) {
      std::filesystem::remove(files.front(), error);
      files.erase(files.begin());
    }
  }

  void record_sample(
    const ImuSample & sample, bool quaternion_valid, const char * drop_reason)
  {
    if (!imu_record_enabled_ || !recording_running_) {
      return;
    }
    const auto now = std::chrono::steady_clock::now();
    double arrival_gap_ms = 0.0;
    if (last_record_time_.has_value()) {
      arrival_gap_ms = std::chrono::duration<double, std::milli>(
        now - *last_record_time_).count();
    }
    last_record_time_ = now;

    {
      std::lock_guard<std::mutex> lock(record_queue_mutex_);
      if (record_queue_.size() >= record_queue_capacity_) {
        ++dropped_record_count_;
        return;
      }
      record_queue_.push_back(
        RecordedSample{sample, arrival_gap_ms, quaternion_valid, drop_reason});
    }
    record_queue_condition_.notify_one();
  }

  void record_writer_loop()
  {
    while (true) {
      RecordedSample recorded_sample;
      {
        std::unique_lock<std::mutex> lock(record_queue_mutex_);
        record_queue_condition_.wait(lock, [this]() {
          return !record_queue_.empty() || !recording_running_;
        });
        if (record_queue_.empty() && !recording_running_) {
          return;
        }
        recorded_sample = std::move(record_queue_.front());
        record_queue_.pop_front();
      }
      write_recorded_sample(recorded_sample);
    }
  }

  void write_recorded_sample(const RecordedSample & recorded_sample)
  {
    if (!record_file_.is_open()) {
      return;
    }
    const auto & message = recorded_sample.sample.imu;
    const double norm = quaternion_norm(message.orientation);
    std::ostringstream row;
    row << std::setprecision(12)
        << std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::system_clock::now().time_since_epoch()).count() << ","
        << message.header.stamp.sec << "," << message.header.stamp.nanosec << ","
        << (recorded_sample.quaternion_valid ? 1 : 0) << ","
        << recorded_sample.drop_reason << ","
        << message.orientation.w << "," << message.orientation.x << ","
        << message.orientation.y << "," << message.orientation.z << "," << norm << ","
        << recorded_sample.sample.euler.vector.x << ","
        << recorded_sample.sample.euler.vector.y << ","
        << recorded_sample.sample.euler.vector.z << ","
        << message.angular_velocity.x << "," << message.angular_velocity.y << ","
        << message.angular_velocity.z << "," << message.linear_acceleration.x << ","
        << message.linear_acceleration.y << "," << message.linear_acceleration.z << ","
        << recorded_sample.arrival_gap_ms << "\n";
    const std::string content = row.str();
    record_file_ << content;
    if (record_file_.fail()) {
      RCLCPP_WARN(get_logger(), "IMU CSV write failed; disabling data recording");
      recording_running_ = false;
      return;
    }
    if (++recorded_rows_since_flush_ >= 20) {
      record_file_.flush();
      if (record_file_.fail()) {
        RCLCPP_WARN(get_logger(), "IMU CSV flush failed; disabling data recording");
        recording_running_ = false;
        return;
      }
      recorded_rows_since_flush_ = 0;
    }
  }

  void close_imu_recorder()
  {
    {
      std::lock_guard<std::mutex> lock(record_queue_mutex_);
      recording_running_ = false;
    }
    record_queue_condition_.notify_one();
    if (record_writer_thread_.joinable()) {
      record_writer_thread_.join();
    }
    if (record_file_.is_open()) {
      record_file_.flush();
      record_file_.close();
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
    const auto mapping = parse_axis_mapping(axis_mapping_);
    if (!mapping.has_value()) {
      if (axis_mapping_ != "identity") {
        RCLCPP_WARN_ONCE(
          get_logger(),
          "unsupported axis_mapping='%s'; using identity mapping",
          axis_mapping_.c_str());
      }
      return;
    }

    const auto rotate = [&mapping](double x, double y, double z) {
      const std::array<double, 3> input{x, y, z};
      return std::array<double, 3>{
        mapping->signs[0] * input[mapping->indices[0]],
        mapping->signs[1] * input[mapping->indices[1]],
        mapping->signs[2] * input[mapping->indices[2]]};
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

    // The orientation uses the inverse coordinate transform on the right,
    // matching the convention used by the existing fixed-axis mapping.
    std::array<std::array<double, 3>, 3> inverse_matrix{};
    for (std::size_t row = 0; row < 3; ++row) {
      inverse_matrix[mapping->indices[row]][row] = mapping->signs[row];
    }
    const auto mapping_quaternion = quaternion_from_matrix(inverse_matrix);
    sample.imu.orientation = multiply_quaternions(
      sample.imu.orientation, mapping_quaternion);

      // Recompute Euler angles from the transformed quaternion. Roll, pitch,
      // and yaw cannot be remapped by simply swapping their three components.
      const auto & quaternion = sample.imu.orientation;
      const double sin_roll = 2.0 *
        (quaternion.w * quaternion.x + quaternion.y * quaternion.z);
      const double cos_roll = 1.0 - 2.0 *
        (quaternion.x * quaternion.x + quaternion.y * quaternion.y);
      const double sin_pitch = 2.0 *
        (quaternion.w * quaternion.y - quaternion.z * quaternion.x);
      const double clamped_sin_pitch = std::clamp(sin_pitch, -1.0, 1.0);
      const double sin_yaw = 2.0 *
        (quaternion.w * quaternion.z + quaternion.x * quaternion.y);
      const double cos_yaw = 1.0 - 2.0 *
        (quaternion.y * quaternion.y + quaternion.z * quaternion.z);
      sample.euler.vector.x = std::atan2(sin_roll, cos_roll);
      sample.euler.vector.y = std::asin(clamped_sin_pitch);
      sample.euler.vector.z = std::atan2(sin_yaw, cos_yaw);
  }

  struct AxisMapping
  {
    std::array<int, 3> indices{};
    std::array<double, 3> signs{};
  };

  static std::optional<AxisMapping> parse_axis_mapping(const std::string & value)
  {
    if (value == "identity") {
      return std::nullopt;
    }

    std::stringstream fields(value);
    std::string token;
    AxisMapping mapping;
    std::array<bool, 3> used{};
    int determinant = 1;
    for (std::size_t row = 0; row < 3; ++row) {
      if (!std::getline(fields, token, ',')) {
        return std::nullopt;
      }
      if (token.empty()) {
        return std::nullopt;
      }
      double sign = 1.0;
      std::size_t axis_position = 0;
      if (token.front() == '-') {
        sign = -1.0;
        axis_position = 1;
      } else if (token.front() == '+') {
        axis_position = 1;
      }
      if (token.size() != axis_position + 1) {
        return std::nullopt;
      }
      const char axis = token[axis_position];
      const int index = axis == 'x' ? 0 : axis == 'y' ? 1 : axis == 'z' ? 2 : -1;
      if (index < 0 || used[index]) {
        return std::nullopt;
      }
      mapping.indices[row] = index;
      mapping.signs[row] = sign;
      used[index] = true;
    }
    if (std::getline(fields, token, ',')) {
      return std::nullopt;
    }

    // A quaternion can represent rotations only, not reflections.
    if (mapping.indices == std::array<int, 3>{0, 1, 2}) {
      determinant *= 1;
    } else if (mapping.indices == std::array<int, 3>{1, 2, 0} ||
      mapping.indices == std::array<int, 3>{2, 0, 1}) {
      determinant *= 1;
    } else {
      determinant *= -1;
    }
    for (const double sign : mapping.signs) {
      determinant = static_cast<int>(determinant * sign);
    }
    if (determinant < 0) {
      return std::nullopt;
    }
    return mapping;
  }

  static geometry_msgs::msg::Quaternion quaternion_from_matrix(
    const std::array<std::array<double, 3>, 3> & matrix)
  {
    geometry_msgs::msg::Quaternion quaternion;
    const double trace = matrix[0][0] + matrix[1][1] + matrix[2][2];
    if (trace > 0.0) {
      const double scale = 0.5 / std::sqrt(trace + 1.0);
      quaternion.w = 0.25 / scale;
      quaternion.x = (matrix[2][1] - matrix[1][2]) * scale;
      quaternion.y = (matrix[0][2] - matrix[2][0]) * scale;
      quaternion.z = (matrix[1][0] - matrix[0][1]) * scale;
    } else if (matrix[0][0] > matrix[1][1] && matrix[0][0] > matrix[2][2]) {
      const double scale = 2.0 * std::sqrt(1.0 + matrix[0][0] - matrix[1][1] - matrix[2][2]);
      quaternion.w = (matrix[2][1] - matrix[1][2]) / scale;
      quaternion.x = 0.25 * scale;
      quaternion.y = (matrix[0][1] + matrix[1][0]) / scale;
      quaternion.z = (matrix[0][2] + matrix[2][0]) / scale;
    } else if (matrix[1][1] > matrix[2][2]) {
      const double scale = 2.0 * std::sqrt(1.0 + matrix[1][1] - matrix[0][0] - matrix[2][2]);
      quaternion.w = (matrix[0][2] - matrix[2][0]) / scale;
      quaternion.x = (matrix[0][1] + matrix[1][0]) / scale;
      quaternion.y = 0.25 * scale;
      quaternion.z = (matrix[1][2] + matrix[2][1]) / scale;
    } else {
      const double scale = 2.0 * std::sqrt(1.0 + matrix[2][2] - matrix[0][0] - matrix[1][1]);
      quaternion.w = (matrix[1][0] - matrix[0][1]) / scale;
      quaternion.x = (matrix[0][2] + matrix[2][0]) / scale;
      quaternion.y = (matrix[1][2] + matrix[2][1]) / scale;
      quaternion.z = 0.25 * scale;
    }
    return quaternion;
  }

  static geometry_msgs::msg::Quaternion multiply_quaternions(
    const geometry_msgs::msg::Quaternion & left,
    const geometry_msgs::msg::Quaternion & right)
  {
    geometry_msgs::msg::Quaternion result;
    result.w = left.w * right.w - left.x * right.x - left.y * right.y - left.z * right.z;
    result.x = left.w * right.x + left.x * right.w + left.y * right.z - left.z * right.y;
    result.y = left.w * right.y - left.x * right.z + left.y * right.w + left.z * right.x;
    result.z = left.w * right.z + left.x * right.y - left.y * right.x + left.z * right.w;
    return result;
  }

  double expected_period_ms() const
  {
    return 1000.0 / imu_frequency_hz_;
  }

  double timeout_period_ms() const
  {
    return expected_period_ms() * imu_timeout_multiplier_;
  }

  void check_imu_timeout(bool sample_received)
  {
    if (imu_frequency_hz_ <= 0.0 || imu_timeout_multiplier_ < 1.0) {
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
  double imu_timeout_multiplier_{1.5};
  bool imu_record_enabled_{false};
  std::string imu_record_enabled_override_{"auto"};
  std::string imu_record_dir_{"/var/log/bxi_log/imu/data"};
  int imu_record_max_files_{10};
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
  std::ofstream record_file_;
  std::string record_file_prefix_;
  std::filesystem::path current_record_path_;
  int record_file_index_{0};
  std::uint64_t recorded_rows_since_flush_{0};
  std::optional<std::chrono::steady_clock::time_point> last_record_time_;
  static constexpr std::size_t record_queue_capacity_{2000};
  std::deque<RecordedSample> record_queue_;
  std::mutex record_queue_mutex_;
  std::condition_variable record_queue_condition_;
  std::atomic<bool> recording_running_{false};
  std::uint64_t dropped_record_count_{0};
  std::thread record_writer_thread_;

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
