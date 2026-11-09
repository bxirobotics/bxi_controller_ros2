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

## Automatic IMU selection

Both modules use `/dev/ttyIMU`. Startup queries HiPNUC model responses or
checksummed Yesense product replies at 921600, 115200, then 460800 baud.
USB identity alone does not identify the protocol. Device parameters are read
back and only mismatched settings are saved: 921600 baud for both, Yesense
200 Hz, and HiPNUC HI91 500 Hz. Correct settings are not written again.
Configuration rejection or readback failure stops startup. The `baudrate`
parameter does not override this fixed device target.

Identity queries first sweep all three baud rates with 100 ms response windows,
then retry with 500 ms windows only if none matched. Fixed waits between baud
rates are removed; native USB CDC retains one 150 ms settling delay after open.
HiPNUC baud-change and reboot ACKs, which may be lost, wait at most 100 ms;
other configuration/readback timeouts and reboot settling remain unchanged.

The selected decoder must then produce three consecutive valid protocol
samples before publication. Module axis mappings and sample validation remain
in effect. `probe_timeout_ms` and `probe_min_frames` control this final data
probe, not identity queries. Custom deployments may still use `fallback_ports`.

`axis_mapping` lists the source axis for target `x,y,z`, with an optional
`-` sign. For example, `"-y,x,z"` means `robot_x=-imu_y`, `robot_y=imu_x`,
and `robot_z=imu_z`. Each source axis must be used once in a right-handed rotation.

Install the rules, link helpers and boot scan together from the repository root:

```bash
bash script/deploy_device_links.sh
ls -l /dev/ttyIMU
```

This backs up and moves the old `bxi-dev.rules` out of the rules directory.
The IMU rule matches `10c4:ea60` (ttyUSB) or `5953:5543` (ttyACM), without
architecture or USB-port restrictions. `bxi-imu-link` keeps one valid target,
selects another matching device on removal, and removes the alias if none remain.
The enabled oneshot service restores the mapping on boot; udev handles hotplug.
See [device-link details](../../script/README.md).
Remapping does not reopen an already running reader; restart the IMU process
through the normal stop/start procedure after reconnecting the device.

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
ros2 launch bxi_imu imu.launch.py driver:=yesense port:=/dev/ttyIMU baudrate:=921600
```

### Controlling IMU CSV recording

The default recording setting is stored in the module configuration:

```text
src/bxi_imu/modules/hipnuc/config.yaml
src/bxi_imu/modules/yesense/config.yaml
```

```yaml
imu_record_enabled: true
imu_record_dir: /var/log/bxi_log/imu/data
imu_record_max_files: 10
```

For a standalone launch, override recording without editing YAML:

```bash
# Disable CSV recording
ros2 launch bxi_imu imu.launch.py imu_record_enabled:=false

# Enable CSV recording
ros2 launch bxi_imu imu.launch.py imu_record_enabled:=true
```

The default `imu_record_enabled:=auto` uses the selected module's own
`config.yaml`, so Hipnuc and Yesense can keep independent recording policies.

When the controller starts `bxi_imu` automatically, edit
`imu_record_enabled` in the active module's `config.yaml`, rebuild, and restart
the controller:

```yaml
imu_record_enabled: false
```

CSV files are written to `imu_record_dir`; startup logs remain under
`/var/log/bxi_log/imu/`.

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
