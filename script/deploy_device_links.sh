#!/usr/bin/env bash
set -euo pipefail

script_name="${BASH_SOURCE[0]##*/}"
printf '[进入脚本] %s\n' "$script_name" >&2
trap '
    script_status=$?
    printf "[离开脚本] %s (退出码=%s)\n" "$script_name" "$script_status" >&2
    exit "$script_status"
' EXIT

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
(($# == 0)) || { echo "usage: deploy_device_links.sh (no arguments)" >&2; exit 2; }
[[ "$(uname -s)" == Linux ]] || { echo 'device links require Linux' >&2; exit 1; }
for command_name in sudo install udevadm flock systemctl timeout; do
    command -v "$command_name" >/dev/null || { echo "missing $command_name" >&2; exit 1; }
done
for name in bxi-battle-dragon-link bxi-imu-link bxi-imu-link.service udev/70-bxi-{bat,imu,remote}.rules; do
    [[ -f "$script_dir/$name" ]] || { echo "missing $script_dir/$name" >&2; exit 1; }
done

sudo mkdir -p /usr/local/bin
sudo install -m 0755 "$script_dir/"bxi-{battle-dragon,imu}-link /usr/local/bin/
sudo install -m 0644 "$script_dir/bxi-imu-link.service" /etc/systemd/system/
sudo install -m 0644 "$script_dir/udev/"70-bxi-{bat,imu,remote}.rules /etc/udev/rules.d/
if [[ -e /etc/udev/rules.d/bxi-dev.rules || -L /etc/udev/rules.d/bxi-dev.rules ]]; then
    backup="$(sudo mktemp -d /var/backups/bxi-udev.XXXXXX)"
    sudo mv -- /etc/udev/rules.d/bxi-dev.rules "$backup/"
    echo "previous device rules retained at $backup/bxi-dev.rules"
fi
sudo udevadm control --reload-rules
sudo timeout 20s udevadm trigger
sudo timeout 20s udevadm settle
sudo systemctl daemon-reload
sudo systemctl enable bxi-imu-link.service
sudo systemctl restart bxi-imu-link.service
