// Keep checks enabled in Release colcon builds as well.
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <string>
#include "remote_controller/imu_receive_diagnostics.hpp"

int main()
{
  remote_controller::ImuReceiveDiagnostics stats;
  assert(!stats.has_sample());
  assert(stats.silence_ms(1) == 0.0);
  stats.observe(1000000000LL, 10000000000LL, 9999000000LL, 0, 0);
  stats.observe(1002000000LL, 10002000000LL, 10001000000LL, 0, 0, false);
  const auto first = stats.report(1003000000LL, 10003000000LL, true);
  assert(first.find("\"window_received\":2") != std::string::npos);
  assert(first.find("\"total_invalid\":1") != std::string::npos);
  assert(first.find("\"max_receive_gap_ms\":2.000") != std::string::npos);
  assert(first.find("\"rmw_timestamps_available\":false") != std::string::npos);
  assert(first.find("\"rmw_source_to_receive_ms\":null") != std::string::npos);
  assert(stats.silence_ms(1123000000LL) == 121.0);
  stats.observe(1124000000LL, 10124000000LL, 10122000000LL,
    10122000000LL, 10123000000LL);
  const auto second = stats.report(1125000000LL, 10125000000LL, false);
  assert(second.find("\"window_received\":1") != std::string::npos);
  assert(second.find("\"total_received\":3") != std::string::npos);
  assert(second.find("\"max_receive_gap_ms\":122.000") != std::string::npos);
  assert(second.find("\"rmw_source_to_receive_ms\":1.000") != std::string::npos);
  assert(second.find("\"rmw_receive_to_callback_ms\":1.000") != std::string::npos);
  // Reset preserves the true window boundary, even if the next message is late.
  assert(second.find("\"window_start_unix_ns\":10003000000") != std::string::npos);
  return 0;
}
