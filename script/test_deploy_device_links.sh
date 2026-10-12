#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
command -v bwrap >/dev/null || { echo 'bubblewrap is required' >&2; exit 77; }
test_dir="$(mktemp -d)"
trap 'rm -rf "$test_dir"' EXIT
mkdir -p "$test_dir/"{bin,rules,backups,localbin,source}
cp -a "$script_dir/udev" "$script_dir/bxi-battle-dragon-link" "$test_dir/source/"
printf '#!/bin/bash\n[[ "$1" == -n ]] || exit 1\nshift\nexec "$@"\n' > "$test_dir/bin/sudo"
printf '#!/bin/bash\nprintf "%%s\\n" "$*" >> "$CALLS"\n' > "$test_dir/bin/udevadm"
chmod +x "$test_dir/bin/"*

bwrap --ro-bind / / --unshare-all --die-with-parent \
    --bind "$test_dir" "$test_dir" \
    --bind "$test_dir/rules" /etc/udev/rules.d \
    --bind "$test_dir/backups" /var/backups \
    --bind "$test_dir/localbin" /usr/local/bin \
    --setenv PATH "$test_dir/bin:/usr/bin:/bin" \
    --setenv CALLS "$test_dir/calls" --setenv TEST_DIR "$test_dir" \
    --setenv DEPLOY_SCRIPT "$script_dir/deploy_device_links.sh" \
    --unsetenv BXI_SUDO_PASSWORD --setenv BXI_SUDO_PASSWORD_FILE "$test_dir/no-password" bash <<'CHECKS'
set -euo pipefail
echo 'site custom rules' > /etc/udev/rules.d/bxi-dev.rules
echo 'unrelated rules' > /etc/udev/rules.d/99-local.rules
for name in bxi-battle-dragon-link udev/70-bxi-{bat,imu,remote}.rules; do
    mv "$TEST_DIR/source/$name" "$TEST_DIR/missing"
    if bash "$DEPLOY_SCRIPT" "$TEST_DIR/source"; then exit 1; fi
    [[ ! -e "$CALLS" && ! -e /usr/local/bin/bxi-battle-dragon-link ]]
    grep -qx 'site custom rules' /etc/udev/rules.d/bxi-dev.rules
    mv "$TEST_DIR/missing" "$TEST_DIR/source/$name"
done
for attempt in 1 2; do
    bash "$DEPLOY_SCRIPT" "$TEST_DIR/source"
    for rule in 70-bxi-{bat,imu,remote}.rules; do
        cmp "$TEST_DIR/source/udev/$rule" "/etc/udev/rules.d/$rule"
    done
    [[ -x /usr/local/bin/bxi-battle-dragon-link && ! -e /etc/udev/rules.d/bxi-dev.rules ]]
    backups=(/var/backups/bxi-udev.*/bxi-dev.rules)
    [[ ${#backups[@]} == 1 ]]
    grep -qx 'site custom rules' "${backups[0]}"
    grep -qx 'unrelated rules' /etc/udev/rules.d/99-local.rules
    [[ "$(grep -cx trigger "$CALLS")" == "$attempt" ]]
done
echo 'PASS device deployment: missing inputs, split rules, legacy backup and repeat install'
CHECKS
