# ELF3 动作精度测试

从官方走路状态（normal 或 forward_back）按 LT＋RT，再按 A、X、Y，分别启动 CXK、SONIC 06、SONIC 11。每次运行一个动作，0.3 秒进入，播放结束自动回 normal。中途按 RB＋X 返回 normal，按 RB＋A 请求零力矩。沿用原手柄映射，原悬挂测试独立运行。

## 安装与使用

将整个 `com.bxi.execution_suite` 目录放到 `src/bxi_example_py_elf3/mods/`，按官方方法构建 `bxi_example_py_elf3`，source 安装环境后使用普通仿真或硬件入口。官方加载器读取每个 Mod 目录的 `mod.yaml` 及其中的 Python 入口。卸下此目录、重新构建并重启即可恢复官方程序。

运行环境沿用 BXI ROS 2、communication 消息、NumPy、PyYAML，以及模型使用的 ONNX 和 ONNX Runtime。用 `BXI_CXK_ROOT` 指向含 `assets/cxkv5.npz` 和 `assets/cxkv5_15k.onnx` 的 CXK 目录，用 `BXI_SONIC_ROOT` 指向带原版 `policy.py` 和 `assets` 的 SONIC 模型目录。两个选定的动作参考已随 Mod 提供。模型也可以放在本 Mod 的 `assets/cxk` 和 `assets/sonic`。设置 `BXI_EXECUTION_SUITE=1` 可在启动时预加载三个模型，否则首次选择时加载。请保留现有模型权重与解码器的版本配对。

SONIC 06 使用 `ndance-1`（1034 帧，20.68 秒），SONIC 11 使用 `ndance-2`（769 帧，15.38 秒）。两段均按 50 Hz 回放，录制来源及校验值见 `assets/selection.json`。

## 记录与判定

Mod 自动启动只读记录器。每个动作保留原始事件，结束后在后台生成 `report.html`、`report.json`、`metrics.csv`、`aligned.npz`。输出根目录可用 `ELF3_EVAL_OUTPUT` 指定，硬件默认 `/var/log/bxi_log/execution_eval`，仿真默认 `~/.ros/execution_eval`。每个动作的 `session_*/run_*/report_status.json` 给出报告绝对路径。目录须对控制器用户可写。记录器和分析器继承控制器的标准输出，沿用原有 _elf 日志重定向。

- 模型的解码结果给出 29 维物理目标角。记录器独立订阅最终 ROS 指令、关节位置反馈、温度及状态机信息。
- 依据官方状态观察回调的 ROS 时间和下一周期边界关联真实指令。相邻序号、模型身份、单个发布者和单条指令均需匹配。记录标记或指令丢失、时序含糊时，该周期的模型误差留空。报告始终保留可用的指令与反馈比较。
- 逐关节计算实际减模型、实际减指令、指令减模型。HTML 支持选动作段、关节和显示曲线；重合曲线可以单独显示。默认门槛定义在 `evaluation/elf3_eval/acceptance.py`，仅作筛查。跟踪误差超限记入报告，继续播放。温度、姿态或反馈故障保留原保护逻辑，人工中止单独标记。
- 模型运行中的状态、故障及动作完成信息由 Mod 输出。后台报告再输出精度结论、超限关节和报告路径。原始数据和报告文件与运行日志分开保存。

评测实现复用 elf3-model-execution-eval 的数据合同、分段、指标、记录和报告模板；输入针对官方 BXI Mod 回调及控制器 ROS 输出适配。CXK 复用 BXI 的 GravityIsaaclabV3 解码器，SONIC 复用指定目录的原版解码器。
