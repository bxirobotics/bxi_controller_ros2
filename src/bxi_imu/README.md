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
ros2 launch bxi_imu imu.launch.py driver:=hipnuc port:=/dev/ttyIMU baudrate:=921600
```

When this driver is used together with the robot hardware launch, disable the
legacy hardware IMU reader so only one process opens the HiPNUC serial device:

```bash
ros2 launch bxi_example_py_elf3 example_demo_hw.launch.py enable_imu:=false
ros2 launch bxi_imu imu.launch.py driver:=hipnuc port:=/dev/ttyIMU
```

The legacy hardware reader remains enabled by default for backward
compatibility. The controller's `start` action passes `enable_imu:=false`
automatically, then waits for an actual message on `/hardware/imu_data`.
If hardware still provides data, it keeps the hardware publisher and does not
start this package. If no message arrives and `/dev/ttyIMU` is free, it starts
`bxi_imu`. A topic publisher count alone is not treated as valid IMU data.

## Adding another IMU

1. Add a class derived from `ImuBackend` in `include/bxi_imu` and
   `src/backends`.
2. Convert that device's protocol into `ImuSample` in the backend. Keep units
   consistent with ROS: radians/s, m/s², tesla, and a normalized quaternion.
3. Register the class in `src/backend_factory.cpp` using a new `driver` name.
4. Add backend-specific parameters to the common YAML file only when needed.

The ROS node, topic names, QoS, frame ID handling, and controller integration
remain unchanged for every adapter.
