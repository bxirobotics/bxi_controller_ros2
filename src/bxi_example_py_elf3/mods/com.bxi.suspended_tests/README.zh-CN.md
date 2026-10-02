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
| RB+A | 进入 zero_torque | 无效 |
| RB+Y | 进入 initial_pos | 无效 |
| LB+A | 从 PD 制动进入 recover | 无效 |
| LB+RB+Y | 仿真可从 PD、实机仅从 zero_torque 进入测试待机 | 无效 |
| LB+RB+B | 无效 | 从待机或 X/Y/A 退出到 zero_torque |
| X | 无效 | 悬挂跑动；再按一次停止并回中位 |
| Y | 无效 | 10-20 Hz 扫频振动；再按一次停止并回中位 |
| A | 无效 | 全关节行程测试；再按一次停止并回中位 |
| B | 无效 | 顺序测试：关节行程、5 分钟振动、跑步 |

测试期间只接收 X/Y/A/B 和 LB+RB+B，其他状态按键不进入状态机。
退出测试或因故障离开测试后，遥控状态事件会保持屏蔽，直到 LB、RB、B
全部松开且所有按键槽位回到零；之后新按下的组合键才生效。
该互锁依赖配套遥控映射在任一 LB/RB/B 按住时发送 `btn_8=3`，
`btn_8=1/2` 仍分别表示进入和退出测试。系统 Stop 和测试故障保护不受影响。

进入测试待机后，关节目标角度默认在 3 秒内平滑过渡到测试中位，KP 同期从
0 增至 `JOINT_KP × 1.10`，KD 为 `JOINT_KD × 1.05`。停止 X/Y/A 后的回中位
及待机保持中位也使用这组增益；X/Y/A 正常执行动作时仍使用原始 `JOINT_KP/KD`。
本目录 `mod.yaml` 的 `idle.params` 可设置 `prepare_sec`（3–20 秒）、
`prepare_kp_scale` 和 `center_kd_scale`（均限制在 0.5–1.2 倍）。
`command_limit_slack_deg` 默认允许测试待机回中位命令在原软件关节限位（两端各留
0.02 rad 余量）外最多再超出 10°；日志仍以原限位报告关节、目标角度及超出度数。
X/Y/A 动作命令仍使用原限位；此配置不会改变硬件驱动保护。反馈角度
5° 的保护门槛及超时均未放宽。确认电机通信和悬挂安全后再逐步调整，
不能用更高增益掩盖 `motor_timeout`。
反馈角度进入
5° 范围后，X/Y/A 才能启动；A 测试还会按自身配置的起始角度容差复核。
测试停止后同样需等反馈回中位。
按 B 只在测试待机且回中位准备完成后生效。顺序测试复用 A 的关节行程轨迹，
完成后回中位并等待反馈到位，再执行 300 秒 Y 振动；振动结束回中位后进入
X 跑步轨迹。跑步阶段持续运行，直到手动退出或温度保护触发。
机器人与电机信息分开放在本目录的 `config/` 下：在 `robot_joints.yaml` 的 `joints`
中按关节名填写 `motor_model`；在 `motor_models.yaml` 的 `models` 中按型号填写
`temperature.torque_limit_c` 和 `temperature.shutdown_c`（单位 °C，前者必须小于后者）。
`robot_joints.yaml` 的 `motor_model_counts` 独立声明每种型号应对应的关节数（包含头部关节）；
加载时会核对实际引用数，数量不符、型号漏写或拼写错误会拒绝 B 顺序测试，并在日志中列出
型号及期望/实际数量。修改关节型号时须同步核对该数量表。
关节条目还包含 `position_limit_rad`（来自 `data/elf3.xml` 的仿真/软件角度范围）和
`max_command_torque_nm`（关节级最大命令扭矩，待按实机参数填写）。头部关节在该模型中
没有角度范围，因此暂为 `null`。`motor_models.yaml` 当前只写温度保护；解析器仍支持
可选的 `max_torque_nm` 与 `mit_ranges`（`position_rad`、`velocity_rad_s`、`kp`、`kd`、
`torque_nm` 各为 `[min, max]`），以后有可靠实机规格再添加。不要把 MuJoCo 的
`ctrlrange` 或 URDF 的 `effort` 当成硬件规格填入。这些可选字段会做格式校验，
但控制输出和硬件保护暂不使用它们；当前 B 测试只使用型号映射和温度阈值。
关节型号或型号的 `torque_limit_c` 未填写时，该关节默认以 **90°C** 作为停止阈值；
未填写的 `shutdown_c` 不会自动设定。明确填写的温度值不合法、YAML 格式错误时，
按 B 仍会故障退出至零力矩。配置文件缺失或为空也使用 90°C 默认阈值。
控制节点重启后才会重新加载配置。顺序测试的全部三个阶段均逐名检查
`/hardware/actuator_states` 中发布的每个关节：未配置型号的关节也按 90°C 检查；
温度数据缺失、非数值、超过 0.5 秒未更新，或电机温度达到该型号的 `torque_limit_c` 时，
锁定故障并请求 `zero_torque`。`shutdown_c` 仅记录第二级阈值，当前软件不会执行硬件断电；
硬件保护仍独立生效。实机运行前先确认消息中的关节名、温度单位及两级温度值。
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
实机测试必须可靠地物理悬挂；硬件 launch 的 `release_suspension` 参数不提供
物理悬挂能力。

## 仿真

普通仿真入口在复位时释放虚拟悬挂；测试状态也不会重新固定机身：

```bash
ros2 launch bxi_example_py_elf3 example_launch_demo.launch.py start_remote_controller:=true
```

仿真复位后进入基础控制域的 `pd_brake`，不会自动进入行走 `normal`。
按 **LB+RB+Y** 进入测试待机；完成 3 秒回中位后按 X/Y/A 开始相应测试。
测试会在自由落地的模型上运行，可能失稳或跌倒，只能用于仿真验证。
此入口的遥控器禁止 `system.start/stop`，按 Start
不会拉起实机进程。默认 `start_remote_controller:=false`，适用于已有安全输入源
或只用 ROS 命令测试。不要与实机控制程序或旧独立测试 launch 同时运行。

测试待机或 X/Y/A 中按 **LB+RB+B** 返回零力矩。测试域不接受其他基础模式按键；
测试故障与 IMU 失联仍会触发保护，遥控器全局 Stop 仍可停止进程。
实机不能从 PD 直接进入测试待机。

旧独立测试节点的 CSV 记录及测试启停服务没有迁入状态机；需要采集测试数据时，
请单独录制 ROS 话题，不要为记录数据而同时启动旧测试节点。
