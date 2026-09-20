# bxi_imu

`bxi_imu` is the common ROS 2 IMU package used by the controller. Each vendor
is kept in an independent module directory. The module owns its protocol
decoder and serial configuration; the common node owns selection, validation,
and ROS publication.

## Output contract

Every module publishes:

```text
/hardware/imu_data   sensor_msgs/msg/Imu
```

The node is `/imu_node` and the executable is `bxi_imu/imu_node`. Backends
convert to ROS units before publication: angular velocity in rad/s, linear
acceleration in m/s^2, magnetic field in tesla, and quaternion fields in ROS
`x, y, z, w` order. Non-finite quaternions and quaternions outside the
configured norm range are dropped. The default accepted range is `0.9..1.1`.

## Module layout

```text
modules/
  hipnuc/
    config.yaml
    hipnuc_backend.cpp
    hipnuc_backend.hpp
    lib/hipnuc_vendor/
  yesense/
    config.yaml
    yesense_backend.cpp
    yesense_backend.hpp
    lib/yesense_decoder.*
    lib/yesense_std_out_decoder.*
```

Each `modules/<name>/config.yaml` contains that IMU's driver name, device
alias, baud rate, output topics, and validation settings. There is no shared
The old shared IMU configuration directory is no longer used. The launch file
scans module configurations and creates the candidate list automatically.

## Priority selection

The numeric suffix in a logical serial alias defines priority:

```text
/dev/ttyIMU                    priority 0
/dev/ttyIMU_YESENSE_1          priority 1
/dev/ttyIMU_YESENSE_2          priority 2
```

Lower numbers are tried first. The node opens candidates exclusively. If a
device is absent, busy, or cannot be opened, the next candidate is tried.

Current module settings:

```text
hipnuc  -> /dev/ttyIMU             -> 921600 baud
yesense -> /dev/ttyIMU_YESENSE_1   -> 921600 baud
```

The udev rules must create the same aliases:

```bash
sudo cp script/bxi-dev.rules /etc/udev/rules.d/bxi-dev.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
ls -l /dev/ttyIMU*
```

## Build and run

```bash
cd /home/tim-dxt/bxi_ws/bxi_orin/bxi_controller_ros2
source /opt/ros/humble/setup.bash
colcon build --packages-select bxi_imu --symlink-install --merge-install
source install/setup.bash
ros2 launch bxi_imu imu.launch.py
```

To force a backend:

```bash
ros2 launch bxi_imu imu.launch.py driver:=hipnuc port:=/dev/ttyIMU baudrate:=921600
ros2 launch bxi_imu imu.launch.py driver:=yesense port:=/dev/ttyIMU_YESENSE_1 baudrate:=921600
```

Inspect the active publisher:

```bash
ros2 node list
ros2 topic info -v /hardware/imu_data
ros2 topic echo --once /hardware/imu_data sensor_msgs/msg/Imu
```

## Controller integration

The controller starts `start_imu_if_owned.sh` after launching the hardware
node. The guard first waits for a real `/hardware/imu_data` message. If
`hardware_elf3` is publishing valid data, `bxi_imu` is not started. Otherwise,
the guard checks the serial aliases and starts the launch file, which scans the
module configurations by alias priority. Exclusive serial locking prevents a
second reader from opening the same tty.

The hardware launch should normally disable its legacy IMU reader:

```bash
ros2 launch bxi_example_py_elf3 example_demo_hw.launch.py enable_imu:=false
```

## Add or remove a module

To add an IMU, create `modules/<vendor>/`, add its backend, decoder, vendor
files, and `config.yaml`, add the sources to `CMakeLists.txt`, and register the
driver in `src/backend_factory.cpp`. Add a udev alias with the desired numeric
suffix. To remove one, remove its module sources, factory registration, udev
rule, and module configuration. The common node does not need to change.

Module backends only read and decode their serial protocol. They do not create
ROS nodes or publishers; the common node handles topics, timestamps, frame IDs,
quaternion validation, and shutdown behavior.
