#!/usr/bin/env bash
set -euo pipefail

script_name="${BASH_SOURCE[0]##*/}"
printf '[进入脚本] %s\n' "$script_name" >&2
trap '
    script_status=$?
    if [[ -n "${config:-}" ]]; then rm -f -- "$config" || true; fi
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


script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
template="${1:?usage: deploy_autostart.sh SERVICE_TEMPLATE ROS_DOMAIN_ID}"
domain="${2:?usage: deploy_autostart.sh SERVICE_TEMPLATE ROS_DOMAIN_ID}"
service=ros_elf_launch.service
mode="${3:-deploy}"
[[ $# -le 3 && "$mode" =~ ^(deploy|prepare|start)$ ]] || { echo 'mode must be deploy, prepare or start' >&2; exit 1; }

if [[ "$mode" != start ]]; then
    config="$(mktemp --suffix=.service)"

    bash "$script_dir/generate_ros_config.sh" "$template" "$domain" > "$config"
    SYSTEMD_UNIT_PATH=/usr/lib/systemd/system:/lib/systemd/system systemd-analyze --generators=no verify "$config"
    sudo -v || { echo 'non-interactive sudo authentication failed; configure BXI_SUDO_PASSWORD or /home/bxi/.config/bxi/sudo-password' >&2; exit 1; }
    backup="$(sudo mktemp -d /var/backups/ros-elf-launch.XXXXXX)"
    unit_dirs=(/etc/systemd/system /run/systemd/system /usr/local/lib/systemd/system /usr/lib/systemd/system /lib/systemd/system)
    for unit_dir in "${unit_dirs[@]}"; do
        for old_config in "$unit_dir/$service" "$unit_dir/$service.d" "$unit_dir/$service.wants" "$unit_dir/$service.requires"; do
            if [[ -e "$old_config" || -L "$old_config" ]]; then
                sudo mkdir -p "$backup$unit_dir"
                sudo cp -a -- "$old_config" "$backup$unit_dir/"
            fi
        done
    done
    echo "previous service configuration retained at $backup"
    if systemctl is-active --quiet "$service" || systemctl cat "$service" &>/dev/null; then
        if ! sudo timeout 20s systemctl stop "$service"; then
            echo 'service stop failed; sending SIGKILL' >&2
            sudo systemctl kill -s SIGKILL "$service" || true
            sudo systemctl reset-failed "$service" || true
            if systemctl is-active --quiet "$service"; then
                echo 'service remains active after SIGKILL' >&2
                exit 1
            fi
        fi
        sudo systemctl disable "$service"
    fi
    for unit_dir in "${unit_dirs[@]}"; do
        sudo rm -rf -- "$unit_dir/$service" "$unit_dir/$service.d" "$unit_dir/$service.wants" "$unit_dir/$service.requires"
    done
    sudo systemctl daemon-reload
    sudo install -m 0644 "$config" "/etc/systemd/system/$service"
fi
[[ "$mode" != prepare ]] || exit 0
sudo systemctl daemon-reload
systemctl cat "$service" >/dev/null
sudo systemctl enable "$service"
systemctl is-enabled --quiet "$service"
load_state="$(systemctl show --property=LoadState --value "$service")"
[[ "$load_state" == loaded ]] || { echo "service failed to load: $load_state" >&2; exit 1; }
sudo systemctl reset-failed "$service"
if ! sudo systemctl restart "$service" ||
    ! timeout 30s bash -c 'until systemctl is-active --quiet "$1"; do sleep 1; done' _ "$service" ||
    ! bash -c 'sleep 3; systemctl is-active --quiet "$1"' _ "$service"; then
    sudo systemctl status "$service" --no-pager || true
    sudo journalctl -u "$service" -n 80 --no-pager || true
    exit 1
fi
