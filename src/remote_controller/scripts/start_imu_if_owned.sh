#!/usr/bin/env bash

set -u

log_file="${1:-/var/log/bxi_log/imu_guard.log}"
driver="${IMU_DRIVER:-auto}"
device_link="${IMU_DEVICE:-auto}"
baudrate="${IMU_BAUDRATE:-921600}"
imu_topic="${IMU_TOPIC:-/hardware/imu_data}"
hardware_node="${HARDWARE_NODE:-/hardware_elf3}"
imu_data_wait_seconds="${IMU_DATA_WAIT_SECONDS:-5}"
bxi_imu_pid=""
imu_guard_pid=""

log() {
  # Bash formats this with its built-in printf: startup decisions stay easy to
  # correlate with the ROS logs without spawning date for every log line.
  printf '[%(%Y-%m-%dT%H:%M:%S%z)T] [imu guard] %s\n' -1 "$*"
}

mkdir -p "$(dirname "$log_file")"

# Keep the complete ownership decision and the child ros2 launch output in a
# dedicated imu_*.log file. The outer launcher retains a separate guard log
# only for shell startup errors.
exec > >(tee -a "$log_file" >/dev/null) 2>&1

stop_children() {
  log "received shutdown signal; stopping IMU child processes"
  [ -z "$imu_guard_pid" ] || kill -SIGINT "$imu_guard_pid" 2>/dev/null || true
  [ -z "$bxi_imu_pid" ] || kill -SIGINT "$bxi_imu_pid" 2>/dev/null || true
  [ -z "$imu_guard_pid" ] || wait "$imu_guard_pid" 2>/dev/null || true
  [ -z "$bxi_imu_pid" ] || wait "$bxi_imu_pid" 2>/dev/null || true
  exit 0
}
trap stop_children INT TERM

log "===== IMU startup ====="
log "started_at=$(date --iso-8601=seconds)"
log "driver=$driver"
log "port_link=$device_link"
log "imu_candidate_priority=port suffix order: ttyIMU, ttyIMU_*_1, ttyIMU_*_2, ..."
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
bxi_imu_executable="${bxi_imu_prefix:+$bxi_imu_prefix/lib/bxi_imu/imu_node}"
remote_controller_prefix="$(ros2 pkg prefix remote_controller 2>/dev/null || true)"
imu_publisher_guard="${remote_controller_prefix:+$remote_controller_prefix/lib/remote_controller/imu_publisher_guard}"
log "bxi_imu package found"

if [ -z "$bxi_imu_prefix" ] || [ ! -d "${bxi_imu_prefix:+$bxi_imu_prefix/share/bxi_imu/modules}" ] ||
  [ ! -x "$bxi_imu_executable" ]; then
  log "bxi_imu is not installed completely; refusing to start the fallback"
  exit 1
fi
if [ -z "$remote_controller_prefix" ] || [ ! -x "$imu_publisher_guard" ]; then
  log "imu_publisher_guard is not installed; refusing to start bxi_imu without publisher ownership checks"
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
log "hardware IMU parameter: $imu_parameter"

# A publisher can exist while hardware IMU reading is disabled, and another
# process can publish the same topic. Only hardware_elf3 GID-tagged valid
# frames may claim ownership of the shared IMU topic.
log "checking $hardware_node for valid IMU frames on $imu_topic"
"$imu_publisher_guard" --ros-args \
  -p "imu_topic:=$imu_topic" \
  -p "hardware_node:=$hardware_node" \
  -p min_hardware_frames:=3 \
  -p hardware_frame_window_ms:=500 \
  -p max_wait_ms:=$((imu_data_wait_seconds * 1000))
startup_guard_status=$?
if [ "$startup_guard_status" -eq 10 ]; then
  log "hardware IMU is actively publishing valid frames; not starting bxi_imu"
  exit 0
fi
if [ "$startup_guard_status" -ne 11 ]; then
  log "hardware IMU ownership check failed with status=$startup_guard_status; refusing bxi_imu fallback"
  exit 1
fi
log "hardware IMU did not publish valid frames within ${imu_data_wait_seconds}s; evaluating bxi_imu fallback"

if [ "$device_link" = "auto" ]; then
  available_candidate=0
  for candidate_link in /dev/ttyIMU*; do
    candidate="$(readlink -f "$candidate_link" 2>/dev/null || true)"
    if [ -z "$candidate" ] || [ ! -e "$candidate" ]; then
      log "priority candidate unavailable: $candidate_link"
      continue
    fi
    available_candidate=1
    fuser -s "$candidate"
    fuser_status=$?
    log "priority candidate=$candidate_link resolved_device=$candidate fuser_status=$fuser_status"
    if [ "$fuser_status" -eq 0 ]; then
      fuser -v "$candidate" 2>&1 || true
    elif [ "$fuser_status" -gt 1 ]; then
      log "unable to inspect $candidate; bxi_imu will enforce TIOCEXCL"
    fi
  done
  if [ "$available_candidate" -ne 1 ]; then
    log "no configured IMU candidate is present; refusing to start bxi_imu"
    exit 1
  fi
else
  device="$(readlink -f "$device_link" 2>/dev/null || true)"
  log "resolved_device=$device"
  if [ -z "$device" ] || [ ! -e "$device" ]; then
    log "IMU device $device_link is unavailable; refusing to start bxi_imu"
    exit 1
  fi
  fuser -s "$device"
  fuser_status=$?
  log "fuser_status=$fuser_status for $device"
  if [ "$fuser_status" -eq 0 ]; then
    log "$device is already in use; refusing to start bxi_imu"
    fuser -v "$device" 2>&1 || true
    exit 0
  fi
  if [ "$fuser_status" -gt 1 ]; then
    log "unable to inspect $device; refusing to start bxi_imu"
    exit 1
  fi
fi

log "fallback checks passed: no message on $imu_topic; bxi_imu will apply configured IMU priority"
log "starting bxi_imu launch so module configs are discovered"
ros2 launch bxi_imu imu.launch.py \
  driver:="$driver" \
  port:="$device_link" \
  baudrate:="$baudrate" &
bxi_imu_pid=$!

# A graph publisher alone does not prove that hardware_elf3 produces IMU data.
# The guard checks message Publisher GIDs and exits only after hardware_elf3
# has sent a consecutive burst of actual IMU frames.
log "monitoring hardware publisher activity by GID while bxi_imu is running"
"$imu_publisher_guard" --ros-args \
  -p "imu_topic:=$imu_topic" \
  -p "hardware_node:=$hardware_node" \
  -p min_hardware_frames:=3 \
  -p hardware_frame_window_ms:=500 &
imu_guard_pid=$!

while kill -0 "$bxi_imu_pid" 2>/dev/null; do
  if ! kill -0 "$imu_guard_pid" 2>/dev/null; then
    wait "$imu_guard_pid"
    imu_guard_status=$?
    if [ "$imu_guard_status" -eq 10 ]; then
      log "hardware IMU publisher is actively sending frames; stopping bxi_imu fallback"
      kill -SIGINT "$bxi_imu_pid" 2>/dev/null || true
      wait "$bxi_imu_pid" 2>/dev/null || true
      exit 0
    else
      log "IMU publisher guard exited unexpectedly with status=$imu_guard_status; stopping bxi_imu fallback"
      kill -SIGINT "$bxi_imu_pid" 2>/dev/null || true
      wait "$bxi_imu_pid" 2>/dev/null || true
      exit 1
    fi
  fi
  sleep 0.1
done

kill -SIGINT "$imu_guard_pid" 2>/dev/null || true
wait "$imu_guard_pid" 2>/dev/null || true
wait "$bxi_imu_pid"
