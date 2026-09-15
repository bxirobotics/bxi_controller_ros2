# bxi_imu

`bxi_imu` is the common ROS 2 IMU module used by the controller. Every
hardware adapter publishes the same `sensor_msgs/msg/Imu` topic, so the robot
controller does not need to know which IMU is installed.

## Start

```bash
ros2 launch bxi_imu imu.launch.py
```

The default configuration uses the HiPNUC adapter and publishes:

```text
/hardware/imu_data   sensor_msgs/msg/Imu
```

The adapter can be selected without changing the launch file:

```bash
ros2 launch bxi_imu imu.launch.py driver:=hipnuc port:=/dev/ttyIMU_2 baudrate:=921600
```

## Adding another IMU

1. Add a class derived from `ImuBackend` in `include/bxi_imu` and
   `src/backends`.
2. Convert that device's protocol into `ImuSample` in the backend. Keep units
   consistent with ROS: radians/s, m/s², tesla, and a normalized quaternion.
3. Register the class in `src/backend_factory.cpp` using a new `driver` name.
4. Add backend-specific parameters to the common YAML file only when needed.

The ROS node, topic names, QoS, frame ID handling, and controller integration
remain unchanged for every adapter.
