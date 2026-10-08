#include <gtest/gtest.h>

#include "bxi_imu/sample_freshness.hpp"

using bxi_imu::FreshnessStatus;
using bxi_imu::SampleFreshness;

TEST(SampleFreshnessTest, DetectsRepeatedAndReversedTicks)
{
  SampleFreshness observer;
  const auto start = std::chrono::steady_clock::time_point{};
  EXPECT_EQ(observer.observe(10, 1000, 0, 1, start, 100).status, FreshnessStatus::first);
  EXPECT_EQ(observer.observe(10, 1000, 0, 1, start, 100).status, FreshnessStatus::repeated);
  EXPECT_EQ(observer.observe(9, 1000, 0, 1, start, 100).status, FreshnessStatus::reversed);
}

TEST(SampleFreshnessTest, AcceptsWrapAndUnknownUnits)
{
  SampleFreshness observer;
  const auto start = std::chrono::steady_clock::time_point{};
  constexpr std::uint64_t modulus = std::uint64_t{1} << 32;
  observer.observe(modulus - 2, 1000, modulus, 1, start, 100);
  EXPECT_EQ(
    observer.observe(3, 1000, modulus, 1, start + std::chrono::milliseconds(5), 100).status,
    FreshnessStatus::fresh);
  EXPECT_EQ(observer.observe(std::nullopt, 0, 0, 0, start, 100).status,
    FreshnessStatus::unknown);
  EXPECT_EQ(observer.observe(1, 0, modulus, 3, start, 100).status,
    FreshnessStatus::first);
  EXPECT_EQ(observer.observe(2, 0, modulus, 3, start + std::chrono::seconds(1), 100).status,
    FreshnessStatus::fresh);
}

TEST(SampleFreshnessTest, ReportsRelativeLagNotAbsoluteAge)
{
  SampleFreshness observer;
  const auto start = std::chrono::steady_clock::time_point{};
  observer.observe(100, 1000, 0, 1, start, 100);
  const auto result = observer.observe(120, 1000, 0, 1,
      start + std::chrono::milliseconds(150), 100);
  EXPECT_EQ(result.status, FreshnessStatus::lagging);
  EXPECT_NEAR(result.relative_lag_ms, 130.0, 1e-6);
  EXPECT_EQ(observer.observe(3000, 1000, 0, 1,
      start + std::chrono::seconds(3), 100).status, FreshnessStatus::first);
  EXPECT_EQ(observer.observe(3010, 1000, 0, 1,
      start + std::chrono::milliseconds(3010), 100).status, FreshnessStatus::fresh);
}
