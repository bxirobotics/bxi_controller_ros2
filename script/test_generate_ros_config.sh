#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
test_dir="$(mktemp -d)"
trap 'rm -rf "$test_dir"' EXIT
config="$test_dir/ros_elf_launch.service"

bash "$script_dir/generate_ros_config.sh" "$script_dir/ros_elf_launch.service" 124 > "$config" 2> "$test_dir/output"
grep -Fxq '[进入脚本] generate_ros_config.sh' "$test_dir/output"
grep -Fxq '[离开脚本] generate_ros_config.sh (退出码=0)' "$test_dir/output"
! grep -q '脚本' "$config"
grep -qx 'Environment=ROS_DOMAIN_ID=124' "$config"
for setting in 'KillMode=control-group' 'KillSignal=SIGINT' 'TimeoutStopSec=15s' 'SendSIGKILL=yes'; do
    [[ "$(grep -cx "$setting" "$config")" == 1 ]]
done
SYSTEMD_UNIT_PATH=/usr/lib/systemd/system:/lib/systemd/system systemd-analyze --generators=no verify "$config"
cp "$config" "$test_dir/template.service"
bash "$script_dir/generate_ros_config.sh" "$test_dir/template.service" 008 > "$config"
grep -qx 'Environment=ROS_DOMAIN_ID=8' "$config"
for domain in 0 232; do
    bash "$script_dir/generate_ros_config.sh" "$script_dir/ros_elf_launch.service" "$domain" > "$config"
    grep -qx "Environment=ROS_DOMAIN_ID=$domain" "$config"
done
for domain in 233 999 -1 abc 18446744073709551616; do
    if bash "$script_dir/generate_ros_config.sh" "$script_dir/ros_elf_launch.service" "$domain" > /dev/null 2> "$test_dir/output"; then
        echo "invalid ROS_DOMAIN_ID accepted: $domain" >&2
        exit 1
    fi
    grep -Fxq '[进入脚本] generate_ros_config.sh' "$test_dir/output"
    grep -Fxq '[离开脚本] generate_ros_config.sh (退出码=1)' "$test_dir/output"
done
sed '/^\[Service\]$/d' "$script_dir/ros_elf_launch.service" > "$test_dir/template.service"
if bash "$script_dir/generate_ros_config.sh" "$test_dir/template.service" 124 > /dev/null 2>&1; then
    echo 'template without [Service] accepted' >&2
    exit 1
fi
echo 'ROS config checks passed'
