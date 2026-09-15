#!/usr/bin/env bash

set -u

log_file="${1:-/var/log/bxi_log/imu_guard.log}"
driver="${IMU_DRIVER:-hipnuc}"
device_link="${IMU_DEVICE:-/dev/ttyIMU}"
baudrate="${IMU_BAUDRATE:-921600}"
imu_topic="${IMU_TOPIC:-/hardware/imu_data}"
hardware_node="${HARDWARE_NODE:-/hardware_elf3}"

log() {
  printf '[imu guard] %s\n' "$*"
}

mkdir -p "$(dirname "$log_file")"

# Keep the complete ownership decision and the child ros2 launch output in a
# dedicated imu_*.log file. The outer launcher retains a separate guard log
# only for shell startup errors.
exec > >(tee -a "$log_file" >/dev/null) 2>&1

log "===== IMU startup ====="
log "started_at=$(date --iso-8601=seconds)"
log "driver=$driver"
log "port_link=$device_link"
log "baudrate=$baudrate"
log "imu_topic=$imu_topic"
log "hardware_node=$hardware_node"

bxi_imu_prefix="$(ros2 pkg prefix bxi_imu 2>&1 || true)"
hardware_prefix="$(ros2 pkg prefix hardware_elf3 2>&1 || true)"
log "bxi_imu_prefix=$bxi_imu_prefix"
log "bxi_imu_launch=${bxi_imu_prefix:+$bxi_imu_prefix/share/bxi_imu/launch/imu.launch.py}"
log "bxi_imu_config=${bxi_imu_prefix:+$bxi_imu_prefix/share/bxi_imu/config/imu.yaml}"
log "bxi_imu_executable=${bxi_imu_prefix:+$bxi_imu_prefix/lib/bxi_imu/imu_node}"
log "hardware_elf3_prefix=$hardware_prefix"
log "hardware_elf3_executable=${hardware_prefix:+$hardware_prefix/lib/hardware_elf3/hardware_elf3}"

if ! command -v fuser >/dev/null 2>&1; then
  log "fuser is unavailable; refusing to start bxi_imu without serial ownership detection"
  exit 1
fi

hardware_node_seen=0
# Give hardware_elf3 time to apply enable_imu:=false and open its other
# hardware interfaces before checking the IMU ownership state.
for _ in $(seq 1 100); do
  if ros2 node list 2>/dev/null | grep -Fxq "$hardware_node"; then
    hardware_node_seen=1
    break
  fi
  sleep 0.1
done

if [ "$hardware_node_seen" -ne 1 ]; then
  log "$hardware_node did not appear; refusing to start bxi_imu because IMU ownership is unknown"
  exit 1
fi

imu_parameter="$(ros2 param get "$hardware_node" hardware_config/imu 2>&1 || true)"
log "hardware_config/imu=$imu_parameter"
if ! printf '%s\n' "$imu_parameter" |
  grep -Eiq '(^|[^[:alnum:]_])false([^[:alnum:]_]|$)'; then
  log "hardware IMU switch was not confirmed as false; refusing to start bxi_imu"
  printf '%s\n' "$imu_parameter" | tee -a "$log_file"
  exit 1
fi
log "hardware IMU switch confirmed disabled: hardware_config/imu=false"

device="$(readlink -f "$device_link" 2>/dev/null || true)"
log "resolved_device=$device"
if [ -z "$device" ] || [ ! -e "$device" ]; then
  log "IMU device $device_link is unavailable; refusing to start bxi_imu"
  exit 1
fi

# A publisher means another ROS node already owns the common IMU data path.
# The serial check below catches a reader that has opened the port but has not
# published its first frame yet.
if ros2 topic info "$imu_topic" 2>/dev/null |
  grep -Eq 'Publisher count: [1-9][0-9]*'; then
  log "$imu_topic already has a publisher; refusing to start bxi_imu"
  ros2 topic info -v "$imu_topic" 2>&1 | tee -a "$log_file" || true
  exit 0
fi

fuser -s "$device"
fuser_status=$?
log "fuser_status=$fuser_status for $device"
if [ "$fuser_status" -eq 0 ]; then
  log "$device is already in use; refusing to start bxi_imu"
  fuser -v "$device" 2>&1 | tee -a "$log_file" || true
  exit 0
fi

if [ "$fuser_status" -gt 1 ]; then
  log "unable to inspect $device (fuser status=$fuser_status); refusing to start bxi_imu"
  exit 1
fi

log "ownership checks passed: no publisher on $imu_topic and $device is free"
exec ros2 launch bxi_imu imu.launch.py \
  driver:="$driver" \
  port:="$device_link" \
  baudrate:="$baudrate"
