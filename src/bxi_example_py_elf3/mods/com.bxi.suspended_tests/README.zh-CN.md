# robot_test 状态机与悬挂测试

实机遥控器 Start 或 bt13 启动 `example_demo_hw.launch.py`，由一个
`bxi_example_py_elf3_demo` 节点发布 `hardware/actuators_cmds`。基础状态使用
50 Hz 控制调度器，测试待机、X/Y/A 和 B 顺序测试的三个阶段共用独立的 200 Hz 调度器。
两个调度器在同一运行时锁下交接发布权；非当前调度器暂停周期唤醒，
不能同时发布关节命令。

## 按键

| 按键 | 生效状态 | 动作 |
| --- | --- | --- |
| Start（js14） | 实机遥控器 | 启动硬件、控制器和 BMS；关闭硬件节点 IMU，由 `bxi_imu` 守护脚本按优先级启动 IMU |
| bt13（js13） | 实机遥控器 | 同样启动硬件、控制器和 BMS，但使用硬件节点 IMU，不启动独立 `bxi_imu` |
| Stop（js11） | 实机遥控器 | 停止上述启动的进程 |
| RB+B | `zero_torque`、`normal`、`initial_pos`、`forward_back` | 进入 `pd_brake` |
| RB+X | `pd_brake` | 进入 `normal` |
| RB+A | `normal`、`forward_back`、`lie_down`、`pd_brake`、`initial_pos`、`recover` | 进入 `zero_torque` |
| RB+Y | `normal`、`forward_back`、`pd_brake`、`zero_torque` | 进入 `initial_pos` |
| LB+A | `pd_brake` | 进入 `recover` |
| LB+RB+X | `normal` / `forward_back` | 启动自动前后行走 / 返回 `normal` |
| RT+Y | `normal` | 进入 `lie_down` |
| LB+RB+Y | 实机 `zero_torque`；仿真还可从 `pd_brake` | 进入测试待机，开始回中位准备 |
| X | 测试待机 / X 单项测试 | 启动悬挂跑动 / 结束并回起始姿态 |
| Y | 测试待机 / Y 单项测试 | 启动 10–20 Hz 扫频振动 / 结束并回起始姿态 |
| A | 测试待机 / A 单项测试 | 启动全关节行程测试 / 结束并回起始姿态 |
| B | 测试待机 / B 顺序测试任一阶段 | 启动顺序测试：关节行程、5 分钟振动、跑步 / 回起始姿态后退出到测试待机 |
| LB+RB+B | 测试待机、X/Y/A 或 B 顺序测试任一阶段 | 退出到 `zero_torque` |

X/Y/A 单项测试运行时，只有当前测试键可再次按下以结束并回测试待机；再次启动时
轨迹、扫频计时和关节行程索引都从头开始，不保留上次进度。不能直接切换到其他测试。
B 顺序测试运行时，X/Y/A 不会切换状态；再按 B 会受控回到起始姿态并退出到测试待机。
故障保护仍可立即切换到零力矩。其他基础模式按键在测试域不进入状态机。
退出测试或因故障离开测试后，遥控状态事件会保持屏蔽，直到 LB、RB、B
全部松开且所有按键槽位回到零；之后新按下的组合键才生效。
该互锁依赖配套遥控映射：除已匹配的进入/退出组合外，任一 LB/RB/B 按住时
发送 `btn_8=3`；`btn_8=1/2` 分别表示进入和退出测试。实机系统 Stop 和测试故障
保护不受影响。

进入测试待机后，关节目标角度默认在 3 秒内平滑过渡到测试中位，KP 同期从
0 增至 `JOINT_KP × 1.10`，KD 为 `JOINT_KD × 1.05`。测试结束后的回中位
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
测试结束后同样需等反馈回中位；回位超时会触发零力矩故障。
A 单项和 B 第一阶段从测试中位开始，用 3 秒将所有关节平滑移到 `0 rad`，
确认反馈到位后才开始关节行程轨迹。行程结束后再回到测试中位，即振动的起始姿态。
按 B 只在测试待机且回中位准备完成后生效。顺序测试复用 A 的关节行程轨迹，
完成后回中位并等待反馈到位，再执行 300 秒 Y 振动；振动结束回中位后进入
X 跑步轨迹。跑步阶段持续运行，直到手动退出或温度保护触发。
每次按 B 真正进入自动测试时，日志会以 `FULL B TEST #编号 START` 标记启动，
列出三阶段顺序、关节行程参数和各电机型号的温度阈值；切换阶段时输出进度。
故障退出会输出 `FAILED` 及具体原因；在进入跑步阶段后按 B 受控退出，
或用 `LB+RB+B` 直接退出，输出 `SUCCESS`。若在关节行程或振动阶段人工退出，
则输出 `INCOMPLETE` 和提前退出原因。
跑步阶段目前没有自动完成时长；温度保护触发属于安全故障，不会记为成功。
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
测试停止阈值为该型号的 `torque_limit_c - 15°C`：配置 90°C 时在 75°C 停止，
配置 100°C 时在 85°C 停止。`torque_limit_c` 本身不修改；日志同时显示测试停止值、
扭矩限制值和关节名。`torque_limit_c` 必须大于 15°C。
关节型号或型号的 `torque_limit_c` 未填写时，默认扭矩限制值为 **90°C**，
因此测试在 **75°C** 停止；
未填写的 `shutdown_c` 不会自动设定。明确填写的温度值不合法、YAML 格式错误时，
按 B 仍会故障退出至零力矩。配置文件缺失或为空也使用 75°C 测试停止阈值。
控制节点重启后才会重新加载配置。顺序测试的全部三个阶段均逐名检查
`/hardware/actuator_states` 中发布的每个关节：未配置型号的关节也按 75°C 检查；
温度数据缺失、非数值、超过 0.5 秒未更新，或电机温度达到测试停止阈值时，
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

实机遥控器 Start 使用 `enable_imu:=false` 并由守护脚本选择独立 IMU；
bt13 使用 `enable_imu:=true`，由硬件节点读取 IMU。两种启动键互斥，不应同时启动。
以下手动命令使用独立 IMU 路径，需另起 `bxi_imu` 或守护脚本；否则
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
按 **LB+RB+Y** 进入测试待机；完成 3 秒回中位后按 X/Y/A 开始单项测试，
或按 B 开始整套顺序测试。
测试会在自由落地的模型上运行，可能失稳或跌倒，只能用于仿真验证。
此入口的遥控器禁用所有 `system.*` 操作，Start、bt13 和 Stop 都不会启停实机进程。
默认 `start_remote_controller:=false`，适用于已有安全输入源
或只用 ROS 命令测试。不要与实机控制程序或旧独立测试 launch 同时运行。

测试待机、X/Y/A 或 B 顺序测试中按 **LB+RB+B** 返回零力矩。测试域不接受其他
基础模式按键；测试故障与 IMU 失联仍会触发保护。实机遥控器的 Stop 可停止进程，
但上述仿真入口禁用了这一系统操作。
实机不能从 PD 直接进入测试待机。

旧独立测试节点的 CSV 记录及测试启停服务没有迁入状态机；需要采集测试数据时，
请单独录制 ROS 话题，不要为记录数据而同时启动旧测试节点。
