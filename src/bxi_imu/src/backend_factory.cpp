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

#include <dlfcn.h>

#include <cstdlib>
#include <filesystem>
#include <mutex>
#include <string>
#include <vector>

#include <unistd.h>

#include "bxi_imu/imu_backend.hpp"

namespace bxi_imu
{

namespace
{
using CreateFunction = ImuBackend * (*)(
  const std::string &, int, const rclcpp::Logger &);
using DriverNameFunction = const char * (*)();

struct LoadedModule
{
  void * handle{nullptr};
  std::string driver;
  CreateFunction create{nullptr};
};

std::vector<LoadedModule> & loaded_modules()
{
  static std::vector<LoadedModule> modules;
  return modules;
}

std::mutex & module_mutex()
{
  static std::mutex mutex;
  return mutex;
}

std::vector<std::filesystem::path> module_directories()
{
  std::vector<std::filesystem::path> directories;
  if (const char * configured = std::getenv("BXI_IMU_MODULE_DIR")) {
    directories.emplace_back(configured);
  }

  char executable_path[4096]{};
  const ssize_t length = readlink("/proc/self/exe", executable_path, sizeof(executable_path) - 1);
  if (length > 0) {
    executable_path[length] = '\0';
    directories.emplace_back(std::filesystem::path(executable_path).parent_path() / "modules");
  }

  directories.emplace_back(std::filesystem::current_path() / "modules");
  return directories;
}

void load_modules(const rclcpp::Logger & logger)
{
  std::lock_guard<std::mutex> lock(module_mutex());
  if (!loaded_modules().empty()) {
    return;
  }

  for (const auto & directory : module_directories()) {
    std::error_code error;
    if (!std::filesystem::is_directory(directory, error)) {
      continue;
    }
    for (const auto & entry : std::filesystem::directory_iterator(directory, error)) {
      if (error || !entry.is_regular_file() || entry.path().extension() != ".so") {
        continue;
      }
      void * handle = dlopen(entry.path().c_str(), RTLD_NOW | RTLD_LOCAL);
      if (handle == nullptr) {
        RCLCPP_WARN(logger, "cannot load IMU module %s: %s", entry.path().c_str(), dlerror());
        continue;
      }
      auto driver_name = reinterpret_cast<DriverNameFunction>(
        dlsym(handle, "bxi_imu_driver_name"));
      auto create = reinterpret_cast<CreateFunction>(
        dlsym(handle, "bxi_imu_create_backend"));
      if (driver_name == nullptr || create == nullptr || driver_name() == nullptr) {
        RCLCPP_WARN(logger, "ignoring invalid IMU module %s", entry.path().c_str());
        dlclose(handle);
        continue;
      }
      loaded_modules().push_back({handle, driver_name(), create});
      RCLCPP_INFO(logger, "loaded IMU module '%s' from %s", driver_name(), entry.path().c_str());
    }
  }
}
}  // namespace

BackendPtr create_backend(
  const std::string & driver,
  const std::string & port,
  int baudrate,
  const rclcpp::Logger & logger)
{
  load_modules(logger);
  for (const auto & module : loaded_modules()) {
    if (module.driver == driver) {
      return BackendPtr(module.create(port, baudrate, logger));
    }
  }

  RCLCPP_ERROR(
    logger,
    "unsupported IMU driver '%s'. Add a plugin under src/bxi_imu/modules",
    driver.c_str());
  return nullptr;
}

}  // namespace bxi_imu
