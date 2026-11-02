# ELF3 双腕夹爪模型

## 来源与装配

`source/robot.urdf` 来自 `two_finger_gripper_mimic_new_urdf.zip` 中的夹爪模型，已统一网格路径及手指连杆、关节的命名。装配后的模型位于 `urdf/elf3_gripper.urdf`；原始模型的主动、从动手指连杆均用于左右两侧夹爪，并分别加上 `l_`、`r_` 前缀。

每侧夹爪底座都包含腕部转接件。腕部安装偏移量 `0.08955 m` 是根据网格表面估计的，实际安装前仍需与实物核对。

四个夹爪网格按用途命名：

- `gripper_base.stl`：夹爪壳体。
- `gripper_wrist_adapter.stl`：腕部转接件。
- `gripper_drive_finger.stl`：主动手指。
- `gripper_follower_finger.stl`：从动手指。

左右夹爪共用这四个网格文件。

## 关节与自由度

每侧夹爪有两个滑动关节，只有 `*_gripper_drive_finger_joint` 可独立控制；`*_gripper_follower_finger_joint` 通过 URDF 的 `<mimic>` 关系以相同位移跟随主动关节。`*_gripper_wrist_mount_joint` 是连接腕部与夹爪底座的固定关节。两个手指关节的轴向相反，因此手指会同步张开或闭合。

整个模型有 33 个独立自由度：原机器人 31 个，加上左右夹爪各 1 个。从动手指关节不额外增加独立自由度。主动手指滑动范围为 `0` 至 `0.04 m`。

## MuJoCo

MuJoCo 不直接执行 URDF 的 `<mimic>` 标签。仿真时加载 `urdf/elf3_gripper.xml`：其中为每个从动手指设置了等式约束，并且仅为主动手指设置电机执行器。这个配套模型用于实现夹爪耦合，不替代现有的整机仿真配置。

修改 URDF 后，需要同步更新 `urdf/elf3_gripper.xml`，并重新验证 MuJoCo 中的手指耦合约束。

## RViz 查看

URDF 中的网格使用相对路径，RViz 不能直接解析。专用启动文件会在内存中将路径转换为绝对 `file://` URI，不修改原 URDF；同时将 RViz 固定坐标系设为 `torso_link`，并启用 `use_mimic_tags`，使从动手指不出现独立滑块。

安装 `joint_state_publisher_gui` 和 `rviz2` 后，在 `bxi_controller_ros2` 仓库根目录运行：

```bash
source /opt/ros/humble/setup.bash
ros2 launch "$(pwd)/resources/elf3_dof31_gripper/launch/display.launch.py"
```

在 `Joint State Publisher` 中拖动左右 `*_gripper_drive_finger_joint` 滑块，检查从动手指是否同步移动。RViz 只验证关节运动关系；受力和物理约束需要在 MuJoCo 中测试。

不要使用 `urdf_tutorial display.launch.py model:=...` 查看此模型：该启动方式不会转换相对网格路径，且其默认固定坐标系与本模型不一致。
