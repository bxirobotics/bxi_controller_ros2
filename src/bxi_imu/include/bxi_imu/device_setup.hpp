// Copyright 2026 BXI Robotics
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <string>

#include <rclcpp/logger.hpp>

namespace bxi_imu
{

// Identify by parameter replies, configure only changed values, and verify readback.
// Returns "hipnuc" or "yesense"; throws on identification/configuration failure.
std::string prepare_imu_device(
  const std::string & port, const std::string & driver, const rclcpp::Logger & logger);

}  // namespace bxi_imu
