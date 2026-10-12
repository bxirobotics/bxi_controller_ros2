#!/usr/bin/env bash
set -euo pipefail

script_name="${BASH_SOURCE[0]##*/}"
printf '[进入脚本] %s\n' "$script_name" >&2
trap '
    script_status=$?
    if [[ -n "${temporary:-}" ]]; then rm -f -- "$temporary" || true; fi
    printf "[离开脚本] %s (退出码=%s)\n" "$script_name" "$script_status" >&2
    exit "$script_status"
' EXIT

[[ $# -le 1 && "${1-0}" =~ ^[01]$ ]] || { echo 'usage: generate_robot_config.sh [0|1]' >&2; exit 1; }
head_enabled=false
[[ "${1-0}" == 0 ]] || head_enabled=true
host_name="$(hostname)"
[[ "$host_name" =~ ^[A-Za-z0-9][A-Za-z0-9._-]*$ ]] || { echo 'invalid hostname' >&2; exit 1; }
short_hostname="${host_name%%.*}"
[[ "$short_hostname" =~ ([0-9]+)$ ]] || { echo "hostname lacks numeric suffix: $host_name" >&2; exit 1; }
device_id="${BASH_REMATCH[1]}"
((${#device_id} <= 3)) || { echo "device id out of range: $device_id" >&2; exit 1; }
domain=$((10#$device_id + 30))
((domain <= 232)) || { echo "ROS_DOMAIN_ID out of range: $domain" >&2; exit 1; }
generated_at="$(date --iso-8601=seconds)"
output="${BXI_ROBOT_CONFIG_FILE:-/opt/bxi/robot_config.yaml}"
output_dir="$(dirname -- "$output")"
mkdir -p -- "$output_dir"
temporary="$(mktemp "$output_dir/.robot-config.XXXXXX")"

# ponytail: 外设固定使用 bxi_robot_config_tool main@1fbcfaf 的默认规格，需要选型号时再扩展。
cat > "$temporary" <<EOF
schema_version: 1
robot_id: '$device_id'
ros_domain_id: $domain
hostname: $host_name
generated_at: '$generated_at'
modules:
  head:
    label: Head
    enabled: $head_enabled
  depth_cameras:
    label: Depth cameras
    enabled: true
    quantity: 1
    items:
      - id: realsense_d435i
        model: realsense_d435i
        model_label: Intel RealSense D435i
        position: head_front
        position_label: Head front
        frame_id: camera_link
  lidars:
    label: Lidars
    enabled: false
    quantity: 0
    items: []
  microphones:
    label: Microphones
    enabled: true
    quantity: 1
    items:
      - id: haokai_8_array_omnidirectional_microphone
        model: haokai_8_array_omnidirectional_microphone
        model_label: HAOKAI 8-array omnidirectional microphone
        position: head_front
        position_label: Head front
  speakers:
    label: Speakers
    enabled: true
    quantity: 1
    items:
      - id: yuexin_active_speaker_dc5v
        model: yuexin_active_speaker_dc5v
        model_label: Yuexin active speaker (DC 5V)
        position: torso_front
        position_label: Torso front
EOF
chmod 0644 "$temporary"
mv -fT -- "$temporary" "$output"
echo "Wrote robot config: $output (head=$head_enabled, ROS_DOMAIN_ID=$domain)"
