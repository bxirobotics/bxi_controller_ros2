#!/usr/bin/env bash
set -euo pipefail

script_name="${BASH_SOURCE[0]##*/}"
printf '[进入脚本] %s\n' "$script_name" >&2
trap '
    script_status=$?
    printf "[离开脚本] %s (退出码=%s)\n" "$script_name" "$script_status" >&2
    exit "$script_status"
' EXIT

template="${1:?usage: generate_ros_config.sh SERVICE_TEMPLATE ROS_DOMAIN_ID}"
domain="${2:?usage: generate_ros_config.sh SERVICE_TEMPLATE ROS_DOMAIN_ID}"

[[ "$domain" =~ ^[0-9]{1,3}$ ]] || {
    echo "invalid ROS_DOMAIN_ID: $domain" >&2
    exit 1
}
domain=$((10#$domain))
((domain <= 232)) || {
    echo "invalid ROS_DOMAIN_ID: $domain" >&2
    exit 1
}
grep -qE '^Environment=ROS_DOMAIN_ID=(XX|[0-9]+)$' "$template" && grep -qx '\[Service\]' "$template" || {
    echo "service template lacks [Service] or ROS_DOMAIN_ID" >&2
    exit 1
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
