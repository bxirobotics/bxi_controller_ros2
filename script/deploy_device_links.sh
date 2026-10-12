#!/usr/bin/env bash
set -euo pipefail

script_name="${BASH_SOURCE[0]##*/}"
printf '[进入脚本] %s\n' "$script_name" >&2
trap '
    script_status=$?
    printf "[离开脚本] %s (退出码=%s)\n" "$script_name" "$script_status" >&2
    exit "$script_status"
' EXIT

# Full entries are downloaded alone; keep their sudo handling self-contained.
sudo() {
    if ((EUID == 0)); then
        command sudo -n "$@"
        return
    fi
    local password="${BXI_SUDO_PASSWORD:-}"
    local password_file="${BXI_SUDO_PASSWORD_FILE:-/home/bxi/.config/bxi/sudo-password}"
    if [[ -z "$password" && ( -e "$password_file" || -L "$password_file" ) ]]; then
        [[ -f "$password_file" && ! -L "$password_file" && "$(stat -c '%a:%u' -- "$password_file")" == "600:$EUID" ]] || {
            echo 'sudo password file must be a regular file owned by the current user with mode 0600' >&2
            return 1
        }
        password="$(<"$password_file")"
        [[ -n "$password" ]] || { echo 'sudo password file is empty' >&2; return 1; }
    fi
    if [[ -n "$password" ]]; then
        printf '%s\n' "$password" | command sudo -S -p '' "$@"
    else
        command sudo -n "$@"
    fi
}
export -f sudo


script_dir="${1:?usage: deploy_device_links.sh PROJECT_SCRIPT_DIR}"
for name in bxi-battle-dragon-link udev/70-bxi-{bat,imu,remote}.rules; do
    [[ -f "$script_dir/$name" ]] || { echo "missing $script_dir/$name" >&2; exit 1; }
done

sudo mkdir -p /usr/local/bin
sudo install -m 0755 "$script_dir/bxi-battle-dragon-link" /usr/local/bin/bxi-battle-dragon-link
[[ -x /usr/local/bin/bxi-battle-dragon-link ]] || { echo 'link helper is not executable' >&2; exit 1; }
sudo install -m 0644 "$script_dir/udev/"70-bxi-{bat,imu,remote}.rules /etc/udev/rules.d/
if [[ -e /etc/udev/rules.d/bxi-dev.rules || -L /etc/udev/rules.d/bxi-dev.rules ]]; then
    backup="$(sudo mktemp -d /var/backups/bxi-udev.XXXXXX)"
    sudo mv -- /etc/udev/rules.d/bxi-dev.rules "$backup/"
    echo "previous device rules retained at $backup/bxi-dev.rules"
fi
sudo udevadm control --reload-rules
sudo timeout 20s udevadm trigger
sudo timeout 20s udevadm settle
