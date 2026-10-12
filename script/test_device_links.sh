#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
command -v bwrap >/dev/null || { echo 'bubblewrap is required' >&2; exit 77; }
test_dir="$(mktemp -d)"
trap 'rm -rf "$test_dir"' EXIT
mkdir -p "$test_dir/dev/input" "$test_dir/sys/class/input"

bwrap --ro-bind / / --unshare-all --die-with-parent \
    --bind "$test_dir/dev" /dev --bind "$test_dir/sys" /sys --tmpfs /run \
    --setenv LINK_HELPER "$script_dir/bxi-battle-dragon-link" bash <<'CHECKS'
set -euo pipefail
link=/dev/input/jsBattleDragon
fixture() {
    mkdir -p "/sys/class/input/$1/device/"{id,capabilities}
    touch "/dev/input/$1"
    printf '%s\n' "$2" > "/sys/class/input/$1/device/id/vendor"
    printf '%s\n' "$3" > "/sys/class/input/$1/device/name"
    printf '%s\n' "$4" > "/sys/class/input/$1/device/capabilities/abs"
}
helper() { sh "$LINK_HELPER" "$@"; }
expect_target() { [[ "$(readlink "$link")" == "$1" ]]; }

ln -s /dev/input/missing "$link"
helper
[[ ! -L "$link" ]]
echo 'PASS device_link no_devices_clears_stale_link'

fixture js0 0000 'Battle Dragon' 30627
fixture js1 20d6 'Other controller' 30627
fixture js2 20d6 'Battle Dragon' 00000
for node in js0 js1 js2 missing; do helper add "$node"; done
[[ ! -L "$link" ]]
echo 'PASS device_link rejects_wrong_vendor_name_axes_and_missing_device'

fixture js0 20D6 'Bluetooth Battle Dragon' 30627
helper add js0
expect_target /dev/input/js0
echo 'PASS device_link accepts_matching_controller'

fixture js1 20d6 'Battle Dragon' 30627
helper add js1
expect_target /dev/input/js0
echo 'PASS device_link preserves_first_valid_controller'

helper remove js2
expect_target /dev/input/js0
echo 'PASS device_link unrelated_removal_preserves_link'

helper remove js0
expect_target /dev/input/js1
echo 'PASS device_link removal_selects_another_controller'

rm /dev/input/js0
helper remove js1
[[ ! -L "$link" ]]
echo 'PASS device_link last_controller_removal_clears_link'

helper
expect_target /dev/input/js1
echo 'PASS device_link rescan_finds_controller'

ln -sfn js1 "$link"
helper
expect_target js1
echo 'PASS device_link accepts_valid_relative_link'

ln -sfn /dev/input/js2 "$link"
helper
expect_target /dev/input/js1
echo 'PASS device_link repairs_invalid_target'
CHECKS
