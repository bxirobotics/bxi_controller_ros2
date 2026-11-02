// Copyright 2026 BXI Robotics
// Observation-only, fixed-size receive window for the independent IMU guard.
#pragma once

#include <algorithm>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <string>

namespace remote_controller
{
class ImuReceiveDiagnostics
{
public:
  void observe(
    std::int64_t steady_ns, std::int64_t wall_ns, std::int64_t header_ns,
    std::int64_t source_ns, std::int64_t received_ns, bool valid = true)
  {
    if (!has_sample_) {
      window_wall_ns_ = wall_ns;
    }
    if (has_sample_) {
      max_gap_ns_ = std::max(max_gap_ns_, steady_ns - last_steady_ns_);
    }
    has_sample_ = true;
    last_steady_ns_ = steady_ns;
    last_wall_ns_ = wall_ns;
    ++count_;
    ++total_;
    if (!valid) {++invalid_;}
    header_age_ms_ = header_ns > 0 ? (wall_ns - header_ns) / 1e6 : 0.0;
    has_header_ = header_ns > 0;
    has_rmw_times_ = source_ns > 0 && received_ns > 0;
    if (has_rmw_times_) {
      source_to_receive_ms_ = (received_ns - source_ns) / 1e6;
      receive_to_callback_ms_ = (wall_ns - received_ns) / 1e6;
    }
  }

  bool has_sample() const {return has_sample_;}

  double silence_ms(std::int64_t now_ns) const
  {
    return has_sample_ ? std::max<std::int64_t>(0, now_ns - last_steady_ns_) / 1e6 : 0.0;
  }

  std::string report(std::int64_t now_ns, std::int64_t wall_ns, bool reset)
  {
    std::ostringstream out;
    out << std::fixed << std::setprecision(3)
        << "{\"window_start_unix_ns\":" << window_wall_ns_
        << ",\"window_end_unix_ns\":" << wall_ns
        << ",\"last_receive_unix_ns\":" << last_wall_ns_
        << ",\"window_received\":" << count_
        << ",\"total_received\":" << total_
        << ",\"total_invalid\":" << invalid_
        << ",\"max_receive_gap_ms\":" << max_gap_ns_ / 1e6
        << ",\"silence_ms\":" << silence_ms(now_ns)
        << ",\"last_header_age_ms\":";
    if (has_header_) {out << header_age_ms_;} else {out << "null";}
    out << ",\"rmw_timestamps_available\":" << (has_rmw_times_ ? "true" : "false")
        << ",\"rmw_source_to_receive_ms\":";
    if (has_rmw_times_) {out << source_to_receive_ms_;} else {out << "null";}
    out << ",\"rmw_receive_to_callback_ms\":";
    if (has_rmw_times_) {out << receive_to_callback_ms_;} else {out << "null";}
    out << '}';
    if (reset) {
      count_ = 0;
      max_gap_ns_ = 0;
      window_wall_ns_ = wall_ns;
    }
    return out.str();
  }

private:
  bool has_sample_{false};
  bool has_header_{false};
  bool has_rmw_times_{false};
  std::uint64_t count_{0};
  std::uint64_t total_{0};
  std::uint64_t invalid_{0};
  std::int64_t window_wall_ns_{0};
  std::int64_t last_wall_ns_{0};
  std::int64_t last_steady_ns_{0};
  std::int64_t max_gap_ns_{0};
  double header_age_ms_{0};
  double source_to_receive_ms_{0};
  double receive_to_callback_ms_{0};
};
}  // namespace remote_controller
