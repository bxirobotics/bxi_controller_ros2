# robot_test 状态机与悬挂测试

遥控器 Start 启动 `example_demo_hw.launch.py`，由一个
`bxi_example_py_elf3_demo` 节点发布 `hardware/actuators_cmds`。基础状态使用
50 Hz 控制调度器，悬挂测试待机、X/Y/A 三个状态共用独立的 200 Hz 调度器。
两个调度器在同一运行时锁下交接发布权；非当前调度器暂停周期唤醒，
不能同时发布关节命令。

## 按键

| 按键 | 基础控制域 | 测试控制域 |
| --- | --- | --- |
| RB+B | 从零力矩进入 PD 制动 | 无效 |
| RB+X | 从 PD 制动进入 normal | 无效 |
| RB+A | 进入 zero_torque | 从任意测试状态立即进入 zero_torque |
| RB+Y | 进入 initial_pos | 无效 |
| LB+A | 从 PD 制动进入 recover | 无效 |
| LB+RB+Y | 仅从 zero_torque 进入测试待机 | 仅从测试待机退出到 zero_torque |
| X | 无效 | 悬挂跑动；再按一次停止并回中位 |
| Y | 无效 | 10-20 Hz 扫频振动；再按一次停止并回中位 |
| A | 无效 | 全关节行程测试；再按一次停止并回中位 |

进入测试待机后，关节命令在 10 秒内平滑进入测试中位。反馈角度进入
0.1 rad 范围后，X/Y/A 才能启动。测试停止后同样需等反馈回中位。
关节反馈超过 0.2 秒未更新、控制周期中断超过 50 ms，或命令越过软件限位时，测试故障锁定并请求
`zero_torque`；需要重启控制程序才能重新进入测试模式。IMU 失联由
`imu_protection` 接管，默认不可切换到其他状态。

当前 A 测试沿用原有配置：启用关节限位、反馈新鲜度与位置误差检查，
MuJoCo 在线碰撞检查未开启。测试必须在机器人可靠悬挂、周围无人时进行。
行程速度、余量与保持时间配置位于本目录 `mod.yaml` 的
`whole_body_joint_test.params`，默认与原 `config/suspended_tests.yaml` 相同。

## 启动与检查

遥控器 Start 会以 `enable_imu:=false` 启动硬件和控制节点，再由 IMU
守护脚本选择独立 IMU。手动启动时，需另起 `bxi_imu` 或守护脚本；否则
控制节点会一直等待 `/hardware/imu_data`：

```bash
ros2 launch bxi_example_py_elf3 example_demo_hw.launch.py enable_imu:=false
ros2 topic info -v /hardware/imu_data
ros2 topic info -v /hardware/actuators_cmds
```

`/hardware/actuators_cmds` 必须只有一个发布者。硬件 IMU 关闭时仍可能
在 ROS 图中保留 `/hardware/imu_data` 的发布端点，因此 IMU 应以守护脚本
的 GID 活动帧检测结果为准，不能只看 publisher 数量。不要同时启动旧的
`example_launch_suspended_tests_hw.launch.py`，它仍是独立的关节命令发布者。
统一硬件 launch 将 `release_suspension` 设为 `false`，保持悬挂测试设置。

旧独立测试节点的 CSV 记录及测试启停服务没有迁入状态机；需要采集测试数据时，
请单独录制 ROS 话题，不要为记录数据而同时启动旧测试节点。
