# 自动前后行走

仅在 `normal` 状态按 **LB+RB+X** 可进入 `forward_back`；再次按相同组合键
返回 `normal`。该状态复用 normal 的行走策略，以 50 Hz 更新速度指令，默认先
以 `+0.3` 前进 2 秒，再以 `-0.5` 后退 2 秒，持续循环。侧移与转向恒为 0；
遥控器摇杆和 `/cmd_vel` 不会改变这一状态的速度指令。

遥控器按钮仍会进入状态机：**RB+A** 可进入零力矩，**RB+B** 可进入 PD
制动，**RB+Y** 可进入初始位置。IMU 失联保护与 normal 相同。测试时应
留出足够的前后空间，并有人监护。

速度与每段时长在本目录 `mod.yaml` 的 `forward_back.params` 中配置：

```yaml
params: {speed: 0.3, backward_speed: 0.5, segment_sec: 2.0}
```

这里的数值是行走策略的期望速度指令，不保证实测位移或速度恰好等于该值。

## 躺下

沿用主分支原按键 **RT+Y**，仅可从 `normal` 进入 `lie_down`。该状态按
`lie_down.npz` 与 `lie_down.onnx` 播放动作，结束后自动过渡到 `pd_brake`；
**RB+A** 仍可紧急切换到零力矩。启动前确认机器人前方 3 米无障碍物，
且躺下后的 PD 制动不等于自动起身。
