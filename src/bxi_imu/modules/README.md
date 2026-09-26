# bxi_imu Modules

Each directory under `modules` is an independent IMU backend. Add a directory
with `config.yaml`, backend source files, and optional vendor libraries. The
common node and backend factory do not need to be edited.

The backend must implement `bxi_imu::ImuBackend` and export:

```cpp
extern "C" const char * bxi_imu_driver_name();
extern "C" bxi_imu::ImuBackend * bxi_imu_create_backend(
  const std::string &, int, const rclcpp::Logger &);
```

The driver name must match `config.yaml`. The launch file discovers all module
configs automatically, and CMake builds every module as a shared plugin.

See [README.zh-CN.md](README.zh-CN.md) for the complete contract.
