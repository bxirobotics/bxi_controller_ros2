// Copyright 2026 BXI Robotics
//
// Monitor one named publisher on a shared IMU topic. The process exits with
// code 10 only after that publisher has produced a small consecutive burst.

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <memory>
#include <set>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace
{
using Gid = std::array<std::uint8_t, RMW_GID_STORAGE_SIZE>;

class ImuPublisherGuard final : public rclcpp::Node
{
public:
  ImuPublisherGuard()
  : Node("imu_publisher_guard")
  {
    topic_ = declare_parameter<std::string>("imu_topic", "/hardware/imu_data");
    hardware_node_ = declare_parameter<std::string>("hardware_node", "/hardware_elf3");
    min_frames_ = declare_parameter<int>("min_hardware_frames", 3);
    frame_window_ms_ = declare_parameter<int>("hardware_frame_window_ms", 500);
    max_wait_ms_ = declare_parameter<int>("max_wait_ms", 0);
    quaternion_norm_tolerance_ = declare_parameter<double>(
      "quaternion_norm_tolerance", 0.1);
    min_frames_ = std::max(min_frames_, 1);
    frame_window_ms_ = std::max(frame_window_ms_, 1);
    max_wait_ms_ = std::max(max_wait_ms_, 0);
    if (!std::isfinite(quaternion_norm_tolerance_) ||
      quaternion_norm_tolerance_ < 0.0 || quaternion_norm_tolerance_ >= 1.0)
    {
      quaternion_norm_tolerance_ = 0.1;
    }

    subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      topic_, rclcpp::SensorDataQoS(),
      [this](
        const sensor_msgs::msg::Imu::ConstSharedPtr & message,
        const rclcpp::MessageInfo & message_info)
      {
        on_imu_frame(*message, message_info);
      });
    graph_timer_ = create_wall_timer(
      std::chrono::milliseconds(100), [this]() {refresh_hardware_gids();});
    refresh_hardware_gids();
    RCLCPP_INFO(
      get_logger(),
      "monitoring %s for active frames from %s; require %d frames within %d ms",
      topic_.c_str(), hardware_node_.c_str(), min_frames_, frame_window_ms_);
  }

  bool hardware_is_active() const
  {
    return hardware_active_;
  }

  bool timed_out() const
  {
    return max_wait_ms_ > 0 &&
           std::chrono::steady_clock::now() - started_at_ >=
           std::chrono::milliseconds(max_wait_ms_);
  }

private:
  static std::string full_node_name(const rclcpp::TopicEndpointInfo & endpoint)
  {
    const auto & node_namespace = endpoint.node_namespace();
    if (node_namespace.empty() || node_namespace == "/") {
      return "/" + endpoint.node_name();
    }
    return node_namespace + "/" + endpoint.node_name();
  }

  void refresh_hardware_gids()
  {
    std::set<Gid> new_gids;
    for (const auto & endpoint : get_publishers_info_by_topic(topic_)) {
      if (full_node_name(endpoint) == hardware_node_) {
        new_gids.insert(endpoint.endpoint_gid());
      }
    }
    if (new_gids != hardware_gids_) {
      hardware_gids_ = std::move(new_gids);
      RCLCPP_INFO(
        get_logger(), "hardware publisher endpoints for %s: %zu",
        hardware_node_.c_str(), hardware_gids_.size());
    }
  }

  bool valid_imu_frame(const sensor_msgs::msg::Imu & message) const
  {
    const auto & orientation = message.orientation;
    if (!std::isfinite(orientation.w) || !std::isfinite(orientation.x) ||
      !std::isfinite(orientation.y) || !std::isfinite(orientation.z))
    {
      return false;
    }
    const double norm = std::sqrt(
      orientation.w * orientation.w + orientation.x * orientation.x +
      orientation.y * orientation.y + orientation.z * orientation.z);
    return norm >= 1.0 - quaternion_norm_tolerance_ &&
           norm <= 1.0 + quaternion_norm_tolerance_;
  }

  void on_imu_frame(
    const sensor_msgs::msg::Imu & message,
    const rclcpp::MessageInfo & message_info)
  {
    if (hardware_active_ || hardware_gids_.empty()) {
      return;
    }
    const auto & publisher_gid = message_info.get_rmw_message_info().publisher_gid;
    Gid gid{};
    std::copy(
      publisher_gid.data, publisher_gid.data + RMW_GID_STORAGE_SIZE, gid.begin());
    if (hardware_gids_.find(gid) == hardware_gids_.end()) {
      return;
    }
    if (!valid_imu_frame(message)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "ignoring invalid IMU frame from %s while checking publisher ownership",
        hardware_node_.c_str());
      consecutive_hardware_frames_ = 0;
      has_last_hardware_frame_ = false;
      return;
    }

    const auto now = std::chrono::steady_clock::now();
    if (!has_last_hardware_frame_ ||
      now - last_hardware_frame_at_ > std::chrono::milliseconds(frame_window_ms_))
    {
      consecutive_hardware_frames_ = 0;
    }
    last_hardware_frame_at_ = now;
    has_last_hardware_frame_ = true;
    ++consecutive_hardware_frames_;
    if (consecutive_hardware_frames_ < min_frames_) {
      return;
    }

    hardware_active_ = true;
    RCLCPP_WARN(
      get_logger(), "hardware IMU is actively publishing: received %d frames from %s",
      consecutive_hardware_frames_, hardware_node_.c_str());
  }

  std::string topic_;
  std::string hardware_node_;
  int min_frames_{3};
  int frame_window_ms_{500};
  int max_wait_ms_{0};
  double quaternion_norm_tolerance_{0.1};
  std::set<Gid> hardware_gids_;
  std::chrono::steady_clock::time_point last_hardware_frame_at_{};
  bool has_last_hardware_frame_{false};
  int consecutive_hardware_frames_{0};
  bool hardware_active_{false};
  const std::chrono::steady_clock::time_point started_at_{std::chrono::steady_clock::now()};
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr graph_timer_;
};
}  // namespace

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImuPublisherGuard>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  while (rclcpp::ok() && !node->hardware_is_active() && !node->timed_out()) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  const bool hardware_is_active = node->hardware_is_active();
  const bool timed_out = node->timed_out();
  executor.remove_node(node);
  rclcpp::shutdown();
  return hardware_is_active ? 10 : timed_out ? 11 : 0;
}
