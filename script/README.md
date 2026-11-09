# ELF3 部署文件

| 文件 | 用途 |
| --- | --- |
| `download_imu_tool.sh` | 按 Linux x86_64／ARM64 下载 IMU 配置工具，校验 ELF 架构后保存为 `imu-tool`。 |
| `generate_robot_config.sh` | 生成机器人配置；默认无头，参数 `1` 为有头，覆盖 `/opt/bxi/robot_config.yaml`。 |
| `generate_ros_config.sh` | 根据 service 模板和 ROS domain 生成服务配置，输出到标准输出。 |
| `deploy_device_links.sh` | 安装设备规则和手柄链接助手，备份移出旧规则并重载 udev。 |
| `deploy_autostart.sh` | 备份并替换旧服务配置，安装、启用并重启 ROS 服务。 |
| `ros_elf_launch.service` | ROS 遥控器开机自启动的 systemd 服务模板。 |
| `bxi-battle-dragon-link` | 创建和维护 Battle Dragon 手柄的固定设备链接。 |
| `udev/70-bxi-bat.rules` | 为电池管理串口创建 `/dev/ttyBAT` 链接。 |
| `udev/70-bxi-imu.rules` | 为 IMU 串口创建 `/dev/ttyIMU` 或 `/dev/ttyIMU_YESENSE_1` 链接。 |
| `udev/70-bxi-remote.rules` | 在手柄插拔时调用链接助手。 |
| `bxi-dev.rules` | 保留的旧版合并规则；部署脚本使用 `udev/` 下三份规则。 |

下载工具需要 Bash、`curl` 或 `wget`。执行 `./download_imu_tool.sh` 后运行
`./imu-tool`，默认保存到当前目录，也可传一个保存目录。使用前停止其他 IMU
读取进程，避免占用同一串口。
