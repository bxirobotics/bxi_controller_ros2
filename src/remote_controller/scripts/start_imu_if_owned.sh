#!/usr/bin/env bash

set -u

log_file="${1:-/var/log/bxi_log/imu_guard.log}"
driver="${IMU_DRIVER:-hipnuc}"
device_link="${IMU_DEVICE:-/dev/ttyIMU}"
baudrate="${IMU_BAUDRATE:-921600}"
imu_topic="${IMU_TOPIC:-/hardware/imu_data}"
hardware_node="${HARDWARE_NODE:-/hardware_elf3}"
imu_data_wait_seconds="${IMU_DATA_WAIT_SECONDS:-5}"

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
log "imu_data_wait_seconds=$imu_data_wait_seconds"

case "$imu_data_wait_seconds" in
  ''|*[!0-9]*|0)
    log "invalid IMU_DATA_WAIT_SECONDS=$imu_data_wait_seconds; using 5"
    imu_data_wait_seconds=5
    ;;
esac

bxi_imu_prefix="$(ros2 pkg prefix bxi_imu 2>/dev/null || true)"
hardware_prefix="$(ros2 pkg prefix hardware_elf3 2>/dev/null || true)"
bxi_imu_config="${bxi_imu_prefix:+$bxi_imu_prefix/share/bxi_imu/config/imu.yaml}"
bxi_imu_executable="${bxi_imu_prefix:+$bxi_imu_prefix/lib/bxi_imu/imu_node}"
log "bxi_imu_prefix=$bxi_imu_prefix"
log "bxi_imu_config=$bxi_imu_config"
log "bxi_imu_executable=$bxi_imu_executable"
log "hardware_elf3_prefix=$hardware_prefix"
log "hardware_elf3_executable=${hardware_prefix:+$hardware_prefix/lib/hardware_elf3/hardware_elf3}"

if [ -z "$bxi_imu_prefix" ] || [ ! -f "$bxi_imu_config" ] ||
  [ ! -x "$bxi_imu_executable" ]; then
  log "bxi_imu is not installed completely; refusing to start the fallback"
  exit 1
fi

if ! command -v fuser >/dev/null 2>&1 || ! command -v timeout >/dev/null 2>&1; then
  log "fuser or timeout is unavailable; refusing to start bxi_imu without safety checks"
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

# A publisher can exist even when hardware IMU reading is disabled. Decide
# whether the hardware path is usable only after receiving an actual sample.
log "topic discovery result for $imu_topic:"
ros2 topic info -v "$imu_topic" 2>&1 || true

sample_file="$(mktemp /tmp/bxi_imu_sample.XXXXXX)"
sample_received=0
sample_deadline=$((SECONDS + imu_data_wait_seconds))
log "waiting up to ${imu_data_wait_seconds}s for an actual message on $imu_topic"
while [ "$SECONDS" -lt "$sample_deadline" ]; do
  if timeout 1s ros2 topic echo --once \
    --qos-reliability best_effort \
    --qos-durability volatile \
    "$imu_topic" sensor_msgs/msg/Imu >"$sample_file" 2>&1; then
    sample_received=1
    break
  fi
  sleep 0.1
done

if [ "$sample_received" -eq 1 ]; then
  log "hardware IMU data received; keeping the existing publisher and not starting bxi_imu"
  sed 's/^/[imu sample] /' "$sample_file"
  rm -f "$sample_file"
  exit 0
fi
rm -f "$sample_file"
log "no IMU message received within ${imu_data_wait_seconds}s; evaluating bxi_imu fallback"

device="$(readlink -f "$device_link" 2>/dev/null || true)"
log "resolved_device=$device"
if [ -z "$device" ] || [ ! -e "$device" ]; then
  log "IMU device $device_link is unavailable; refusing to start bxi_imu"
  exit 1
fi

# A reader may own the tty without publishing usable data. Never start a
# second reader until the real serial device is confirmed free.
fuser -s "$device"
fuser_status=$?
log "fuser_status=$fuser_status for $device"
if [ "$fuser_status" -eq 0 ]; then
  log "$device is already in use; refusing to start bxi_imu"
  fuser -v "$device" 2>&1 || true
  exit 0
fi

if [ "$fuser_status" -gt 1 ]; then
  log "unable to inspect $device (fuser status=$fuser_status); refusing to start bxi_imu"
  exit 1
fi

log "fallback checks passed: no message on $imu_topic and $device is free"
log "starting bxi_imu executable directly so its exit code is reported"
exec "$bxi_imu_executable" \
  --ros-args \
  --params-file "$bxi_imu_config" \
  -p "driver:=$driver" \
  -p "port:=$device_link" \
  -p "baudrate:=$baudrate"
