# bxi_imu

`bxi_imu` 是控制程序使用的统一 ROS 2 IMU 包。每个厂商的 IMU 都放在
独立模块目录中，模块负责自己的串口协议读取和解析，公共节点负责设备
选择、数据校验和 ROS 话题发布。

## 统一输出

所有 IMU 模块都发布同一个话题：

```text
/hardware/imu_data   sensor_msgs/msg/Imu
```

节点名称和可执行文件为：

```text
/imu_node   （可执行文件：bxi_imu/imu_node）
```

模块输出到 ROS 前会统一转换单位：

- 角速度：弧度每秒 `rad/s`
- 线加速度：`m/s^2`
- 磁场：特斯拉 `tesla`
- 姿态：ROS 使用的四元数 `x, y, z, w`

四元数包含非有限值，或者模长不在配置范围内时，会丢弃该帧。默认允许
的模长范围是 `0.9..1.1`。

## 模块目录

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

每个 `modules/<名称>/config.yaml` 保存该 IMU 的驱动名称、设备软连接、
波特率、输出话题和校验参数。现在已经删除公共的 IMU `config` 目录。

## IMU 自动探测

每个厂商仍由自己的 `modules/<厂商>/config.yaml` 配置。`port` 是首选端口，
`fallback_ports` 可列出备用端口；同一端口上的厂商按 `priority` 从小到大
依次探测。探测要求在限定时间内连续收到 3 帧协议解析成功、四元数及运动
数据有效的帧。未通过时会关闭串口，再尝试下一个候选，不会仅凭串口打开
成功认定型号。

当前顺序是：

```text
/dev/ttyIMU              -> hipnuc  (priority: 0)
/dev/ttyIMU              -> yesense (priority: 1)
/dev/ttyIMU_YESENSE_1    -> yesense (备用端口)
```

先完成所有 `/dev/ttyIMU` 的协议探测，再尝试其他端口；其他端口仍使用
对应模块自己的协议。两个厂商当前均为 921600 波特率。无候选通过时节点
打印逐项失败原因并以错误状态退出，不发布 IMU 数据。启动日志由遥控器的
IMU guard 写入 `/var/log/bxi_log/imu/imu_*.log`。`no_valid_protocol_frames_or_no_data`
表示指定时间内没有可解码帧，单凭这一条不能区分无串口数据与协议不匹配。

可用 `probe_timeout_ms`（默认 1200）及 `probe_min_frames`（默认 3）
调整探测窗口和确认帧数。坐标轴可以在对应模块的 `config.yaml` 中配置，
格式为目标坐标系的
`x,y,z` 分量分别取设备的哪个轴，可加 `-` 表示取反，例如：

```yaml
axis_mapping: "-y,x,z"
```

表示 `robot_x=-imu_y`、`robot_y=imu_x`、`robot_z=imu_z`。三个轴必须各使用一次，且必须构成右手坐标系。

udev 规则必须创建与配置一致的软连接。备用别名不存在时会记录
`open_failed`，不会影响已选中的主端口。修改规则后执行：

```bash
sudo cp script/bxi-dev.rules /etc/udev/rules.d/bxi-dev.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
ls -l /dev/ttyIMU*
```

## 编译和启动

```bash
cd /home/tim-dxt/bxi_ws/bxi_orin/bxi_controller_ros2
source /opt/ros/humble/setup.bash
colcon build --packages-select bxi_imu --symlink-install --merge-install
source install/setup.bash
```

自动按照优先级选择 IMU：

```bash
ros2 launch bxi_imu imu.launch.py
```

强制测试某个模块：

```bash
ros2 launch bxi_imu imu.launch.py \
  driver:=hipnuc port:=/dev/ttyIMU baudrate:=921600

ros2 launch bxi_imu imu.launch.py \
  driver:=yesense port:=/dev/ttyIMU_YESENSE_1 baudrate:=921600
```

### 控制 IMU CSV 记录

默认是否记录由模块配置文件决定：

```text
src/bxi_imu/modules/hipnuc/config.yaml
src/bxi_imu/modules/yesense/config.yaml
```

配置项为：

```yaml
imu_record_enabled: true
imu_record_dir: /var/log/bxi_log/imu/data
imu_record_max_files: 10
```

单独启动 IMU 时，可以用启动参数临时覆盖配置：

```bash
# 关闭 CSV 记录
ros2 launch bxi_imu imu.launch.py imu_record_enabled:=false

# 开启 CSV 记录
ros2 launch bxi_imu imu.launch.py imu_record_enabled:=true
```

默认值 `imu_record_enabled:=auto`，表示采用最终选中 IMU 模块自身
`config.yaml` 中的设置；因此 Hipnuc 和 Yesense 可以独立设置记录策略。

如果由控制程序自动拉起 `bxi_imu`，启动脚本使用模块 YAML 中的配置。
此时需要修改当前模块的 `config.yaml`：

```yaml
imu_record_enabled: false
```

修改后重新编译并重启控制程序。CSV 数据保存在 `imu_record_dir`，启动
日志仍保存在：

```text
/var/log/bxi_log/imu/
```

CSV 的 `receive_time_ns` 在读取线程入队时记录，而不是写盘线程处理时记录。
CSV 还记录设备原始 `device_tick`、单位 `device_tick_period_us`、`device_frame_id`、
`freshness_status`、`frame_status` 和 `relative_lag_ms`。状态有 `unknown`、`first`、
`fresh`、`repeated`、`reversed`、`lagging`。`relative_lag_ms` 是最近两秒
窗口内主机与设备时间增量的差值，**不是绝对采样年龄**。超核 HI91 使用毫秒，
HI83 系统时间使用微秒；元生时间单位可能是微秒或 100 微秒，当前仅记录
原始值及帧号；帧号重复/回退只告警，不用来计算延迟或自动拒收。

默认 `imu_freshness_mode:=observe` 仅限频报告疑似重复、回退或相对滞后，
不改变发布和机器人保护行为。只有实机核对设备计数规律后，才考虑显式
使用 `imu_freshness_mode:=enforce`；该模式只对**已知单位**的设备 tick
拒收异常帧，阈值由 `imu_freshness_lag_limit_ms` 控制（默认 100 ms）。
控制端另行观察 ROS 消息头相对接收时刻的传输年龄；由于消息头在驱动
解码时生成，无法据此发现解码前的串口积压，且观察结果不会触发卸力。
节点每 30 秒输出日志队列的当前深度、峰值和累计丢行数；队列达到
75% 容量时每 5 秒最多警告一次。队列满时只丢 CSV 行，不影响 IMU 话题发布。

查看当前使用的设备和话题发布者：

```bash
ros2 node list
ros2 topic info -v /hardware/imu_data
ros2 topic echo --once /hardware/imu_data sensor_msgs/msg/Imu
```

## 和控制程序的关系

控制程序启动 `hardware_elf3` 后，会执行 `start_imu_if_owned.sh`。脚本先
等待 `/hardware/imu_data` 的真实消息：

- 如果硬件节点已经发布真实 IMU 数据，不启动 `bxi_imu`。
- 如果没有收到真实消息，检查配置的串口软连接。
- 然后启动 `bxi_imu`，由它扫描各模块配置并按软连接优先级选择设备。
- 串口使用独占锁，避免两个程序同时打开同一个设备。

硬件节点通常应该这样启动：

```bash
ros2 launch bxi_example_py_elf3 example_demo_hw.launch.py enable_imu:=false
```

## 新增和删除模块

新增 IMU 时，创建 `modules/<厂商>/`，添加 backend、协议解析器和
`config.yaml`，在 `CMakeLists.txt` 中加入源文件，并在
`src/backend_factory.cpp` 注册驱动名称。再添加对应的 udev 软连接，用
后缀数字表示优先级。

删除 IMU 时，删除模块目录、CMake 源文件、工厂注册项、udev 规则和该模块
配置即可。公共节点不需要修改。

模块 backend 只负责串口读取和协议解析，不创建 ROS 节点和 publisher。
ROS 话题、时间戳、坐标系、四元数校验和退出逻辑都由公共节点负责。
