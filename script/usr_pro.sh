#!/usr/bin/env bash
set -euo pipefail

export PASSWORD=Bxi12345go

sudo() {
    command sudo -S -p '' "$@" <<< "$PASSWORD"
}
export -f sudo

script_name="${BASH_SOURCE[0]##*/}"
printf '[进入脚本] %s\n' "$script_name" >&2
trap '
    script_status=$?
    if [[ -n "${stage:-}" ]]; then rm -rf -- "$stage" || true; fi
    printf "[离开脚本] %s (退出码=%s)\n" "$script_name" "$script_status" >&2
    exit "$script_status"
' EXIT

workspace=/home/bxi/bxi_ws
project="$workspace/bxi_rl_controller_ros2_example"
repository_url="https://download.bxirobotics.cn/d/ELF3/bxi_rl_controller_ros2_example/bxi_ws/bxi_rl_controller_ros2_example.tar.gz"
service=ros_elf_launch.service

trap 'printf "\n已取消部署。\n" >&2; exit 130' INT
printf '\n========== 文件覆盖警告 ==========\n' >&2
printf '接下来将覆盖或替换现有工程、机器人配置、udev 规则和自启动服务配置。\n' >&2
printf '5 秒后自动继续；按 q 或 Ctrl+C 退出（q 无需回车）。\n\n' >&2
wait_until=$((SECONDS + 5))
while ((SECONDS < wait_until)); do
    if IFS= read -r -s -n 1 -t "$((wait_until - SECONDS))" cancel_key; then
        if [[ "$cancel_key" == q || "$cancel_key" == Q ]]; then
            printf '\n已取消部署。\n' >&2
            exit 0
        fi
    else
        remaining=$((wait_until - SECONDS))
        if ((remaining > 0)); then sleep "$remaining"; fi
    fi
done
trap - INT

for command_name in sudo timeout systemctl wget tar grep sed udevadm flock chmod hostname mkdir mktemp realpath bash systemd-analyze cp mv rm install dirname awk; do
    type -P "$command_name" >/dev/null || { echo "missing command: $command_name" >&2; exit 1; }
done
[[ -f /opt/bxi/bxi_ros2_pkg/setup.bash ]] || { echo 'missing bxi_ros2_pkg/setup.bash' >&2; exit 1; }
short_hostname="$(hostname)"
short_hostname="${short_hostname%%.*}"
[[ "$short_hostname" =~ ([0-9]+)$ ]] || { echo "hostname lacks numeric suffix: $short_hostname" >&2; exit 1; }
device_id="${BASH_REMATCH[1]}"
((${#device_id} <= 3)) || { echo "device id out of range: $device_id" >&2; exit 1; }
domain=$((10#$device_id + 30))
((domain <= 232)) || { echo "ROS_DOMAIN_ID out of range: $domain" >&2; exit 1; }

sudo -v
mkdir -p "$workspace" /home/bxi/.cache/bxi-build/deploy /home/bxi/bxi_archive
stage="$(mktemp -d /home/bxi/.cache/bxi-build/deploy/usr-pro.XXXXXX)"
candidate="$stage/bxi_rl_controller_ros2_example"
download_source() {
    wget -O "$stage/source.tar.gz" "$repository_url" &&
        tar -xzf "$stage/source.tar.gz" -C "$stage"
}
if ! download_source; then
    echo "source download failed; removing $project and retrying once" >&2
    if systemctl cat "$service" >/dev/null 2>&1; then
        sudo timeout 20s systemctl stop "$service"
        sudo systemctl disable "$service"
    fi
    sudo rm -rf -- "$project" "$candidate"
    download_source
fi
for name in build.sh script/ros_elf_launch.service script/udev/70-bxi-{bat,imu,remote}.rules script/bxi-battle-dragon-link script/bxi-imu-link script/bxi-imu-link.service; do
    [[ -f "$candidate/$name" ]] || { echo "repository lacks $name" >&2; exit 1; }
done

scripts="$candidate/script"
for name in deploy_autostart.sh deploy_device_links.sh generate_ros_config.sh generate_robot_config.sh; do
    [[ -f "$scripts/$name" ]] || { echo "missing $scripts/$name" >&2; exit 1; }
    bash -n "$scripts/$name"
done
robot_config=/opt/bxi/robot_config.yaml
BXI_ROBOT_CONFIG_FILE="$stage/robot_config.yaml" bash "$scripts/generate_robot_config.sh"
bash -n "$candidate/build.sh"
bash "$scripts/generate_ros_config.sh" "$candidate/script/ros_elf_launch.service" "$domain" > "$stage/ros-elf-preflight.service"
SYSTEMD_UNIT_PATH=/usr/lib/systemd/system:/lib/systemd/system systemd-analyze --generators=no verify "$stage/ros-elf-preflight.service"

backup="$(mktemp -d /home/bxi/bxi_archive/pre-deploy.XXXXXX)"
if [[ -e "$robot_config" || -L "$robot_config" ]]; then
    sudo cp -pL -- "$robot_config" "$backup/robot_config.yaml"
fi

BXI_AUTOSTART_MODE=prepare bash "$scripts/deploy_autostart.sh"
if [[ -e "$project" ]]; then
    sudo mv "$project" "$backup/"
    echo "previous project retained at $backup/bxi_rl_controller_ros2_example"
fi
mv "$candidate" "$project"
scripts="$project/script"

bash "$scripts/deploy_device_links.sh"
cd "$project"
sudo rm -rf -- "$project/build" "$project/install" "$project/log"
bash -c '
    source /opt/bxi/bxi_ros2_pkg/setup.bash || exit $?
    printf "[进入脚本] build.sh\n" >&2
    build_status=0
    bash build.sh || build_status=$?
    printf "[离开脚本] build.sh (退出码=%s)\n" "$build_status" >&2
    exit "$build_status"
'
[[ -f install/setup.bash ]] || { echo 'build did not produce install/setup.bash' >&2; exit 1; }
sudo install -m 0644 "$stage/robot_config.yaml" "$robot_config"
echo "deployment backup: $backup"
BXI_AUTOSTART_MODE=start bash "$scripts/deploy_autostart.sh"
echo "usr deployment complete: $project (ROS_DOMAIN_ID=$domain, service=$(systemctl is-active "$service"))"
