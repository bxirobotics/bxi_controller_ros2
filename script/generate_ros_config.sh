#!/usr/bin/env bash

# 供同目录部署脚本 source；加载时不修改调用方的选项、trap 或配置。
load_ros_identity() {
    host_name="$(hostname)" || return
    [[ "$host_name" =~ ^[A-Za-z0-9][A-Za-z0-9._-]*$ ]] || { echo 'invalid hostname' >&2; return 1; }
    local short_hostname="${host_name%%.*}"
    [[ "$short_hostname" =~ ([0-9]+)$ ]] || { echo "hostname lacks numeric suffix: $host_name" >&2; return 1; }
    device_id="${BASH_REMATCH[1]}"
    ((${#device_id} <= 3)) || { echo "device id out of range: $device_id" >&2; return 1; }
    # shortcut: Domain ID 在31～232每202个编号复用，跨机通信需另行隔离或分配。
    domain=$(((10#$device_id + 201) % 202 + 31))
}

generate_ros_config() {
    load_ros_identity || return
    local script_dir
    script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)" || return
    local template="$script_dir/ros_elf_launch.service"
    grep -qE '^Environment=ROS_DOMAIN_ID=(XX|[0-9]+)$' "$template" && grep -qx '\[Service\]' "$template" || {
        echo "service template lacks [Service] or ROS_DOMAIN_ID: $template" >&2
        return 1
    }
    awk -v domain="$domain" '
        /^(KillMode|KillSignal|TimeoutStopSec|SendSIGKILL)=/ { next }
        /^\[Service\]$/ {
            print
            print "KillMode=control-group\nKillSignal=SIGINT\nTimeoutStopSec=15s\nSendSIGKILL=yes"
            next
        }
        /^Environment=ROS_DOMAIN_ID=(XX|[0-9]+)$/ { $0 = "Environment=ROS_DOMAIN_ID=" domain }
        { print }
    ' "$template"
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
    set -euo pipefail
    script_name="${BASH_SOURCE[0]##*/}"
    printf '[进入脚本] %s\n' "$script_name" >&2
    trap '
        script_status=$?
        printf "[离开脚本] %s (退出码=%s)\n" "$script_name" "$script_status" >&2
        exit "$script_status"
    ' EXIT
    (($# == 0)) || { echo 'usage: generate_ros_config.sh (no arguments)' >&2; exit 2; }
    generate_ros_config
fi
