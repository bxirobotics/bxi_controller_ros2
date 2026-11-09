# ELF3 部署文件

| 文件 | 用途 |
| --- | --- |
| `usr_pro.sh` | 无参数下载并部署正式版源码包。 |
| `test_pro.sh` | 无参数下载并部署测试版源码包。 |
| `download_imu_tool.sh` | 按 Linux x86_64／ARM64 下载 IMU 配置工具，校验 ELF 架构后保存为 `imu-tool`。 |
| `generate_robot_config.sh` | 生成机器人配置；默认无头，参数 `1` 为有头，覆盖 `/opt/bxi/robot_config.yaml`。 |
| `generate_ros_config.sh` | 根据 service 模板和 ROS domain 生成服务配置，输出到标准输出。 |
| `deploy_device_links.sh` | 安装设备规则和链接助手，备份移出旧规则、重载 udev 并启用 IMU 开机扫描。 |
| `deploy_autostart.sh` | 备份并替换旧服务配置，安装、启用并重启 ROS 服务。 |
| `ros_elf_launch.service` | ROS 遥控器开机自启动的 systemd 服务模板。 |
| `bxi-battle-dragon-link` | 创建和维护 Battle Dragon 手柄的固定设备链接。 |
| `bxi-imu-link` | 按 udev 标记扫描 IMU，维护唯一的 `/dev/ttyIMU` 链接。 |
| `bxi-imu-link.service` | 开机扫描已有 IMU，在 ROS 遥控服务之前运行。 |
| `udev/70-bxi-bat.rules` | 为电池管理串口创建 `/dev/ttyBAT` 链接。 |
| `udev/70-bxi-imu.rules` | 不限制架构或 USB 口，标记匹配 IMU 并在插拔和 change 事件时更新映射。 |
| `udev/70-bxi-remote.rules` | 在手柄插拔时调用链接助手。 |

两个部署模块自动使用脚本同目录文件，不需要传路径或 domain：

```bash
cd script
./deploy_device_links.sh
./deploy_autostart.sh
```

`deploy_autostart.sh` 按主机名末尾编号加 30 计算 ROS domain，例如
`elf3-124` 为 154；无编号或超出 232 时退出。usr/test 内部通过
`BXI_AUTOSTART_MODE=prepare/start` 分别在构建前安装 unit、构建后启动服务。
usr/test沿用旧入口的内置密码自动完成sudo验证，并将提权函数传给子Bash脚本，
构建后再次提权也无需手动输入；会替换工程、规则和服务，入口保留5秒取消窗口。
旧工程归档到 `/home/bxi/bxi_archive/`，下载暂存到 `/home/bxi/.cache/bxi-build/deploy/`。

AMD64、ARM64 和测试分支发布 CI 将两个入口覆盖上传到下载站的
`bxi_ws/usr_pro.sh`、`robot_test/test_pro.sh`。IMU 依赖 `udevadm`、`flock` 和 systemd；
多匹配设备保留当前有效映射，串口读取进程在拔插后仍需 Stop/Start 重新打开串口。

`bxi_imu` 启动时按参数协议识别元生（Yesense）／超核（HiPNUC），探测
921600、115200、460800，读回配置后只修改不符的参数：统一 921600，
元生 200 Hz、超核 HI91 500 Hz；配置验证失败时退出。udev 助手只维护映射，
不会在插拔事件内发送配置命令。

独立检查或人工配置可运行（不要与 IMU 读取进程同时使用）：

```bash
./download_imu_tool.sh
./imu-tool
```

下载工具需要 Bash、`curl` 或 `wget`；默认保存到当前目录，也可传一个保存目录。
驱动自身完成自动识别与配置，不依赖下载工具。
