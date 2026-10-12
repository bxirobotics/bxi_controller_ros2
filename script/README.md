# ELF3 部署功能脚本

`elf3-scripts` 的 `usr/usr_pro.sh`、`robot_test/test_pro.sh` 负责下载、校验和构建项目；本目录负责机器人硬件配置、设备链接与自启动配置。

| 脚本 | 参数 | 行为 |
| --- | --- | --- |
| `generate_robot_config.sh` | 无参数或 `0`：无头；`1`：有头 | 纯 Bash 生成 `/opt/bxi/robot_config.yaml`，无需交互；外设使用参考工具的默认规格 |
| `generate_ros_config.sh` | service 模板、ROS_DOMAIN_ID | 配置写到 stdout，不修改系统；domain 限定为 0–232，最多 3 位十进制数；模板支持 `XX` 或已有数字 |
| `deploy_device_links.sh` | 项目 script 目录 | 安装手柄链接助手与三份 udev 规则、备份移出旧规则、重载并触发全部设备 |
| `deploy_autostart.sh` | service 模板、ROS_DOMAIN_ID | 校验新配置，备份并清理旧 unit/override/依赖配置，安装并启用/重启服务 |

适用环境：Linux、Bash、GNU coreutils、systemd、udev，已安装 ELF3 ROS 2 环境。机器人配置生成只依赖 Bash 和 GNU coreutils，不依赖 Python/PyYAML。系统修改使用非交互 sudo：读取独立凭据文件或环境变量自动认证；未提供凭据时使用 `sudo -n`，认证失败直接退出。脚本不保存密码。完整部署需要 `git` 和项目构建依赖。

两个完整部署入口和四个功能脚本均向 stderr 输出 `[进入脚本] 文件名`、`[离开脚本] 文件名 (退出码=N)`；成功为 0，失败保留原退出码，临时文件按原逻辑清理。完整入口调用 `build.sh` 时也输出对应标记。日志不写入 stdout，因此生成的 systemd 配置不含日志；两个 switch 保留原有输出，不修改。强制终止（SIGKILL）或断电无法执行离开日志。

## 使用

在设备上的项目根目录执行：

```bash
# 默认无头，写入正式配置；已有文件会被替换。
sudo bash script/generate_robot_config.sh
# 显式无头 / 有头。
sudo bash script/generate_robot_config.sh 0
sudo bash script/generate_robot_config.sh 1

# 只生成示例，不改正式配置。
BXI_ROBOT_CONFIG_FILE=/tmp/robot_config.yaml bash script/generate_robot_config.sh

# 94 号设备：94 + 30 = 124；这条命令生成 systemd service 配置。
bash script/generate_ros_config.sh script/ros_elf_launch.service 124 > /tmp/ros_elf_launch.service
SYSTEMD_UNIT_PATH=/usr/lib/systemd/system:/lib/systemd/system systemd-analyze --generators=no verify /tmp/ros_elf_launch.service

# 以下两条修改系统；自启动脚本会启动 ROS 进程，应先确保硬件安全。
bash script/deploy_device_links.sh "$PWD/script"
bash script/deploy_autostart.sh "$PWD/script/ros_elf_launch.service" 124
```

机器人硬件配置的 YAML 格式与外设默认规格参考 [bxi_robot_config_tool main@1fbcfaf](https://github.com/timtimting/bxi_robot_config_tool/tree/1fbcfafcea91543d89fe4ce258b7d4ae0ac1d060)（2026-10-09 对照），包含主机信息、`modules.head.enabled`，以及深度相机、激光雷达、麦克风、扬声器的数量、型号、位置和附加字段。头部只有布尔开关；默认改为无头。外设固定为 1 台 D435i、0 台 Livox、1 个 HAOKAI 麦克风、1 个悦欣音箱。配置清单不代表实测外设在线，改头部开关也不会自动删除相机/麦克风/音箱清单。

脚本无参数等同 `0`，只接受单个 `0` 或 `1`，非法参数在写入前退出；`BXI_ROBOT_CONFIG_FILE` 可覆盖输出位置，默认 `/opt/bxi/robot_config.yaml`。通过同目录临时文件和原子替换写入，失败保留旧文件。独立运行会覆盖已有配置，若需要保留自定义字段，应先自行备份。

两个完整部署入口每次都先调用无参数生成器，在停止旧服务前生成暂存的无头配置。旧配置和旧项目保存到同一个 `.pre-deploy.XXXXXX/` 备份目录；构建成功后安装新配置，再启用/重启 service。所以切换 usr/test 会重新生成无头配置，即使先前的配置是有头。完整入口没有头部参数；需要有头时，部署完成后单独执行生成器 `1`，在硬件程序下次启动时读取。

`deploy_device_links.sh` 先检查 `script/udev/70-bxi-{bat,imu,remote}.rules` 和手柄链接助手齐全，再安装三份规则；已有 `/etc/udev/rules.d/bxi-dev.rules` 移到唯一的 `/var/backups/bxi-udev.XXXXXX/` 目录，避免旧规则重复生效。随后重载规则，执行无子系统限制的 `udevadm trigger` 和 `settle`（各限时 20 秒）。三份同名规则每次覆盖，其他规则保留；需要保留的现场自定义内容须先合入源码，旧规则备份不会自动恢复。`BXI_REPLACE_UDEV_RULES` 不再影响行为。

service 模板必须包含 `[Service]` 和独立的 `Environment=ROS_DOMAIN_ID=XX` 或数字行。停止策略统一为 `KillMode=control-group`、`KillSignal=SIGINT`、`TimeoutStopSec=15s`、`SendSIGKILL=yes`。模板中的工作区路径需匹配设备实际安装路径。

重新部署 service 时，先把旧配置备份到 `/var/backups/ros-elf-launch.XXXXXX/`，停止并禁用旧服务，然后删除各 systemd unit 目录中的 `ros_elf_launch.service`、同名 `.d`、`.wants`、`.requires` 目录，再安装新 unit、`daemon-reload`、启用服务，用 `systemctl show` 确认 `LoadState=loaded` 后执行 `reset-failed`、重启。systemd 249 会回收未启用且无引用的 unit；仅 show 后直接 reset-failed 仍可能报 `Unit not loaded`，所以先 enable 挂到活动的启动 target。扫描范围为 `/etc/systemd/system`、`/run/systemd/system`、`/usr/local/lib/systemd/system`、`/usr/lib/systemd/system`、`/lib/systemd/system`；只清理该服务的同名配置。原有 override 和依赖设置会失效，备份不会自动恢复。usr/test 都通过这个脚本执行清理，两个 switch 文件不修改。

完整入口预检查使用临时 unit 名 `ros-elf-preflight.service`，避免尚未清理的旧同名 drop-in 干扰新模板校验。完整入口先调用 `deploy_autostart.sh 模板 domain prepare`，备份并清理旧配置、安装新 unit；构建成功后调用相同脚本的 `start` 阶段，重载、启用和重启。省略第三参数仍完整执行安装与启动。构建失败时服务保持禁用，新 unit 已安装，备份保留。

模板预检用 `SYSTEMD_UNIT_PATH=/usr/lib/systemd/system:/lib/systemd/system` 和 `--generators=no`，加载基础系统依赖，不扫描现场 `/etc`、`/run` 配置或运行生成器，避免无关 netplan 权限和旧 systemd 对 snapd 新字段的警告。新 unit 安装后再通过系统管理器的实际 LoadState 与启用/运行状态验证，不忽略加载或启动失败。

## sudo 无交互认证

完整入口及设备链接/自启动模块使用相同的 sudo 处理，并传给 Bash 子脚本。root 执行时直接使用 `sudo -n`，普通用户认证优先使用环境变量 `BXI_SUDO_PASSWORD`，其次读取 `BXI_SUDO_PASSWORD_FILE` 指定的文件；默认文件为 `/home/bxi/.config/bxi/sudo-password`。文件只存一行密码，必须由当前执行用户拥有、权限为 `0600`、为普通文件且不是软链接；文件为空或权限不符会退出。凭据存在时通过 stdin 传给 `sudo -S -p ''`，后续每次提权也采用相同方式，避免构建较久导致缓存过期后要求输入。

没有凭据时使用 `sudo -n`，已有免密权限或有效认证缓存即可执行；权限不足时直接报错。设备准备阶段需配置独立凭据文件或免密 sudo，之后正常运行原 switch 无需输入密码。独立凭据文件含明文密码，应保留在设备的私有目录，不能放进源码、tar.gz、版本管理或日志；脚本不改 sudoers，也不添加全局免密授权。

## Git 部署与失败处理

usr/test 均通过 `https://v4.gh-proxy.org/https://github.com/bxirobotics/bxi_controller_ros2.git` 克隆默认分支到临时目录。默认使用仓库内的四个功能脚本；需要临时验证外部版本时，把本目录（包含 `generate_robot_config.sh`）放到待替换项目之外，例如 `/home/bxi/bxi_controller_ros2/script`，并在运行完整部署入口时设置：

```bash
BXI_DEPLOY_SCRIPTS_DIR=/home/bxi/bxi_controller_ros2/script bash usr/usr_pro.sh
```

入口会解析真实路径，拒绝指向待替换项目内部的路径、软链接或 `..` 别名。完整入口提前检查系统命令、ROS underlay 并检查非交互 sudo 认证。首次克隆失败时，先停止并禁用旧服务，再删除固定项目目录和残留克隆目录，重试一次；停服务失败直接退出并保留旧工程。**第二次克隆仍失败时退出，旧工程已删除，服务保持停用。**首次克隆成功时先预检，再停止并禁用旧服务，旧项目保留在输出的 `.pre-deploy.XXXXXX/` 目录。正常替换阶段 stop 超时/失败后发送 SIGKILL、尝试 reset-failed 并检查是否仍 active，仍运行就退出，不替换项目。构建或重启失败不会自动回滚或重新启动旧版；重复部署会增加备份占用空间。

## 验证

```bash
# 只检查语法和配置，不修改系统。
for file in script/*.sh script/bxi-battle-dragon-link; do bash -n "$file" || exit 1; done
bash script/test_generate_ros_config.sh
python3 script/test_generate_robot_config.py
# 需要 bubblewrap；隔离 /dev、/sys、/run，不操作真实手柄。
bash script/test_device_links.sh
bash script/test_deploy_device_links.sh
```

生成测试用 Python/PyYAML 解析 YAML 作断言，这只是测试依赖，生成器运行不需要它们。流程回归位于 `elf3-scripts/test/test_deployment.sh`，需要 `bubblewrap`；它运行真实部署脚本，通过隔离文件系统和模拟服务/udev/构建覆盖正常部署、旧包兼容、规则覆盖与全设备触发、停服务失败后的强杀兜底、缺命令提前退出、失败退出和旧项目备份，不代表真实机器人启动或运动测试通过。

`test_device_links.sh` 用模拟 sysfs 和设备文件测试手柄识别、重复添加、移除切换、最后一只移除、重新扫描和失效链接修复，共 10 项；不代表真实 USB/蓝牙连接验收。

两个 `switch_*` 入口仍从下载服务器获取对应完整部署脚本后执行；新版完整入口通过 Git 获取项目。提交到 `dev` 后由 `Sync Public Main` CI 生成公开树并同步到 `main`；CI 尚未完成或入口尚未发布时，默认远程地址不会自动使用本地修改，可用 `USR_PRO_SCRIPT_URL` / `TEST_PRO_SCRIPT_URL` 指向新版入口，以及外部 `BXI_DEPLOY_SCRIPTS_DIR` 提供未提交的功能模块。两个 switch 原有首参数 `0` 仍表示保留下载脚本，不是头部选择参数。
