# bxi_imu 模块开发说明

`modules` 目录中的每个子目录代表一种 IMU 后端。新增 IMU 时，按下面的目录结构添加一个目录，公共的 `imu_node` 和 `backend_factory.cpp` 不需要修改。

```text
modules/
  new_imu/
    config.yaml
    new_imu_backend.cpp
    new_imu_backend.hpp
    lib/                       # 可选，厂商 SDK 或协议解析库
```

后端文件名也可以使用通用名称 `backend.cpp` 和 `backend.hpp`。

## 后端接口

后端类必须继承 `bxi_imu::ImuBackend`，实现 `open()`、`read()`、`is_open()`、`close()` 和 `name()`。`read()` 成功时填写 `ImuSample`，失败时返回 `false`。

每个模块的 `.cpp` 文件末尾还必须导出两个 C 接口：

```cpp
extern "C" const char * bxi_imu_driver_name()
{
  return "new_imu";
}

extern "C" bxi_imu::ImuBackend * bxi_imu_create_backend(
  const std::string & port, int baudrate, const rclcpp::Logger & logger)
{
  return new NewImuBackend(port, baudrate, logger);
}
```

返回的驱动名称必须与 `config.yaml` 中的 `driver` 完全一致。

## 配置文件

每个模块必须包含 `config.yaml`：

```yaml
imu_module:
  ros__parameters:
    driver: new_imu
    port: /dev/ttyIMU_NEW_1
    baudrate: 921600
    frame_id: imu_link
    imu_topic: /hardware/imu_data
    axis_mapping: "x,y,z"
    imu_frequency_hz: 200.0
    imu_timeout_multiplier: 2.5
    imu_enabled: true
    quaternion_norm_tolerance: 0.1
    euler_enabled: false
    magnetic_enabled: false
    temperature_enabled: false
    pressure_enabled: false
```

启动文件自动扫描 `install/share/bxi_imu/modules/*/config.yaml`，并把模块加入候选列表。端口软链接末尾数字决定优先级：

```text
/dev/ttyIMU       优先级 0
/dev/ttyIMU_1     优先级 1
/dev/ttyIMU_2     优先级 2
```

厂商库放在自己的 `lib/` 目录中。CMake 会自动收集模块目录内的 `.cpp` 和 `.c` 文件，生成对应的共享库，不需要修改公共 CMake 或工厂代码。

## 构建和验证

```bash
source /opt/ros/humble/setup.bash
colcon build --merge-install --packages-select bxi_imu --cmake-force-configure
source install/setup.bash
ros2 launch bxi_imu imu.launch.py
```

日志中应出现：

```text
loaded IMU module 'new_imu' from .../libbxi_imu_new_imu.so
```

所有后端最终使用统一话题：

```text
/hardware/imu_data
```

