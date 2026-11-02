#!/usr/bin/env bash
# Keep the IMU alive until controller/hardware shutdown has completed. This
# helper must also be used by App/API stop paths, rather than killing IMU first.
set -u

control_pattern='[r]os2 launch bxi_example_py_elf3 .*demo_hw.*\.py|[h]ardware_elf3|[b]xi_example_py_elf3_demo|[/]bxi_depth_camera/cameras|[b]xi_depth_camera\.manager'
imu_pattern='[/]start_imu_if_owned[.]sh|[/]bxi_imu|[/]imu_node|[i]mu.launch.py'
aux_pattern='[r]os2 launch bxi_example_bms bms\.launch\.py|[/]bxi_bms|[/]bxi_example_bms|[b]xi-voice'

log() { printf '[%(%Y-%m-%dT%H:%M:%S%z)T] [robot stop] %s\n' -1 "$*"; }

# pgrep/pkill retain the existing command-line matching contract. Never match
# this helper's own name. A bounded INT -> TERM -> KILL escalation replaces the
# previous simultaneous sensor/controller kill and is safe to run repeatedly.
wait_group() {
  local pattern="$1" attempts="$2" i status
  for ((i=0; i<attempts; i++)); do
    pgrep -f "$pattern" >/dev/null
    status=$?
    [ "$status" -eq 1 ] && return 0
    [ "$status" -eq 0 ] || return 2
    sleep 0.1
  done
  pgrep -f "$pattern" >/dev/null
  [ "$?" -eq 1 ]
}
stop_group() {
  local name="$1" pattern="$2"
  log "stopping $name"
  pkill -SIGINT -f "$pattern" || true
  wait_group "$pattern" 20 && return 0
  pkill -SIGTERM -f "$pattern" || true
  wait_group "$pattern" 10 && return 0
  pkill -SIGKILL -f "$pattern" || true
  wait_group "$pattern" 10
}

if ! stop_group 'controller/hardware (IMU remains alive)' "$control_pattern"; then
  log 'ERROR: controller/hardware still present; keeping IMU alive; stop failed'
  stop_group 'auxiliary processes' "$aux_pattern" || true
  exit 1
fi
log 'controller/hardware exited; now stopping IMU'
result=0
stop_group 'IMU' "$imu_pattern" || result=1
stop_group 'auxiliary processes' "$aux_pattern" || result=1
exit "$result"
