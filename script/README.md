# ELF3 部署文件

| 文件 | 用途 |
| --- | --- |
| `download_imu_tool.sh` | 按 Linux x86_64／ARM64 下载 IMU 配置工具，校验 ELF 架构后保存为 `imu-tool`。 |
| `generate_robot_config.sh` | 生成机器人配置；默认无头，参数 `1` 为有头，覆盖 `/opt/bxi/robot_config.yaml`。 |
| `generate_ros_config.sh` | 根据 service 模板和 ROS domain 生成服务配置，输出到标准输出。 |
| `deploy_device_links.sh` | 安装设备规则和链接助手，备份移出旧规则、重载 udev 并启用 IMU 开机扫描。 |
| `deploy_autostart.sh` | 备份并替换旧服务配置，安装、启用并重启 ROS 服务。 |
| `ros_elf_launch.service` | ROS 遥控器开机自启动的 systemd 服务模板。 |
| `bxi-battle-dragon-link` | 创建和维护 Battle Dragon 手柄的固定设备链接。 |
| `bxi-imu-link` | 按 udev 标记维护统一的 `/dev/ttyIMU` 链接。 |
| `bxi-imu-link.service` | 开机扫描已有 IMU，在 ROS 遥控服务之前运行。 |
| `udev/70-bxi-bat.rules` | 为电池管理串口创建 `/dev/ttyBAT` 链接。 |
| `udev/70-bxi-imu.rules` | 不限制架构和 USB 口，在插拔及 change 事件时更新 IMU 映射。 |
| `udev/70-bxi-remote.rules` | 在手柄插拔时调用链接助手。 |
| `bxi-dev.rules` | 保留的旧版合并规则；部署脚本使用 `udev/` 下三份规则。 |

`usr_pro.sh` 和 `test_pro.sh` 均无参数调用 `deploy_device_links.sh`，自动安装
IMU 链接助手、udev 规则和 `bxi-imu-link.service`，重载规则并启用、运行开机
扫描服务；随后通过 `deploy_autostart.sh` 安装并启用 ROS 自启动服务。
多匹配设备保留当前有效映射，拔插后读取进程仍需正常 Stop/Start 重开串口。

下载工具需要 Bash、`curl` 或 `wget`。执行 `./download_imu_tool.sh` 后运行
`./imu-tool`，默认保存到当前目录，也可传一个保存目录。使用前停止其他 IMU
读取进程，避免占用同一串口。
