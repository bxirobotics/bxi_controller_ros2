#pragma once

#include <chrono>
#include <algorithm>
#include <cstdint>
#include <optional>

namespace bxi_imu
{

enum class FreshnessStatus {unknown, first, fresh, repeated, reversed, lagging};

inline const char * freshness_status_name(FreshnessStatus status)
{
  switch (status) {
    case FreshnessStatus::unknown: return "unknown";
    case FreshnessStatus::first: return "first";
    case FreshnessStatus::fresh: return "fresh";
    case FreshnessStatus::repeated: return "repeated";
    case FreshnessStatus::reversed: return "reversed";
    case FreshnessStatus::lagging: return "lagging";
  }
  return "unknown";
}

struct FreshnessResult
{
  FreshnessStatus status{FreshnessStatus::unknown};
  double relative_lag_ms{0.0};
};

class SampleFreshness
{
public:
  FreshnessResult observe(
    std::optional<std::uint64_t> tick, std::uint32_t period_us,
    std::uint64_t modulus, std::uint8_t kind,
    std::chrono::steady_clock::time_point now, double lag_limit_ms)
  {
    if (!tick) {
      reset();
      return {};
    }
    if (!last_tick_ || kind != last_kind_ || period_us != last_period_us_ ||
      modulus != last_modulus_ ||
      (window_start_ && now - *window_start_ >= std::chrono::seconds(2)))
    {
      start(*tick, period_us, modulus, kind, now);
      return {FreshnessStatus::first, 0.0};
    }
    if (*tick == *last_tick_) {
      return {FreshnessStatus::repeated, relative_lag_ms_};
    }
    std::uint64_t delta = 0;
    if (*tick > *last_tick_) {
      delta = *tick - *last_tick_;
    } else if (modulus != 0 && *tick < modulus && *last_tick_ < modulus &&
      (*last_tick_ - *tick) > modulus / 2)
    {
      delta = modulus - *last_tick_ + *tick;
    } else {
      start(*tick, period_us, modulus, kind, now);
      return {FreshnessStatus::reversed, 0.0};
    }
    if (modulus != 0 && delta > modulus / 2) {
      start(*tick, period_us, modulus, kind, now);
      return {FreshnessStatus::reversed, 0.0};
    }
    if (period_us != 0) {
      const double host_delta_ms =
        std::chrono::duration<double, std::milli>(now - *last_time_).count();
      relative_lag_ms_ = std::max(
        0.0, relative_lag_ms_ + host_delta_ms - static_cast<double>(delta) * period_us / 1000.0);
    }
    last_tick_ = *tick;
    last_time_ = now;
    return {
      period_us != 0 && relative_lag_ms_ > lag_limit_ms ?
        FreshnessStatus::lagging : FreshnessStatus::fresh, relative_lag_ms_};
  }

private:
  void reset()
  {
    last_tick_.reset();
    last_time_.reset();
    window_start_.reset();
    relative_lag_ms_ = 0.0;
  }

  void start(
    std::uint64_t tick, std::uint32_t period_us, std::uint64_t modulus,
    std::uint8_t kind, std::chrono::steady_clock::time_point now)
  {
    last_tick_ = tick;
    last_time_ = now;
    window_start_ = now;
    last_period_us_ = period_us;
    last_modulus_ = modulus;
    last_kind_ = kind;
    relative_lag_ms_ = 0.0;
  }

  std::optional<std::uint64_t> last_tick_;
  std::optional<std::chrono::steady_clock::time_point> last_time_;
  std::optional<std::chrono::steady_clock::time_point> window_start_;
  std::uint32_t last_period_us_{0};
  std::uint64_t last_modulus_{0};
  std::uint8_t last_kind_{0};
  double relative_lag_ms_{0.0};
};

}  // namespace bxi_imu
