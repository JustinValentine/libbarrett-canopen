#!/usr/bin/env bash
# Operator-launched torque test with passive CAN capture. No CAN configuration
# or drive settings are changed by this wrapper. The test itself applies torque.
set -euo pipefail
script_dir=$(cd -- "$(dirname -- "$0")" && pwd)
tool=${LBC_TORQUE_SIGN_BIN:-"$script_dir/build/lbc_torque_sign"}
iface=${LBC_CAN_IFACE:-can1}
if [[ ! -x "$tool" ]]; then
    echo "Cannot execute $tool; set LBC_TORQUE_SIGN_BIN to the built torque tool." >&2
    exit 1
fi
command -v candump >/dev/null
command -v stdbuf >/dev/null
if pgrep -u "$UID" -f '[b]t-wam-|[e]x04_display_basic_info|[e]x04_gravity_ramp|[l]bc_torque_sign|[l]bc_position_watch|[w]am8_j8_sweep|[e]x01_initialize_wam|[w]am8g_state' >/dev/null; then
    echo "Another WAM program is running or stopped. Exit it before this test." >&2
    exit 1
fi
mkdir -p "$HOME/sbflash"
log_dir=$(mktemp -d "$HOME/sbflash/torque-diagnostic-$(date +%Y%m%d-%H%M%S)-XXXXXX")
printf 'Command:' > "$log_dir/command.txt"
printf ' %q' "$tool" "$@" >> "$log_dir/command.txt"
printf '\n' >> "$log_dir/command.txt"
ip -details -statistics link show "$iface" > "$log_dir/can-before.txt"
candump -t a -e -d "$iface,0:0,#FFFFFFFF" > "$log_dir/can.log" 2>&1 &
capture_pid=$!
cleanup() {
    kill -INT "$capture_pid" 2>/dev/null || true
    wait "$capture_pid" 2>/dev/null || true
    ip -details -statistics link show "$iface" > "$log_dir/can-after.txt" 2>&1 || true
    echo "Saved diagnostic logs: $log_dir"
}
trap cleanup EXIT
sleep 0.2
if ! kill -0 "$capture_pid" 2>/dev/null; then
    cat "$log_dir/can.log" >&2
    echo "CAN capture failed; torque test was not started." >&2
    exit 1
fi
echo "Recording CAN and terminal output in $log_dir"
set +e
stdbuf -o0 -e0 "$tool" "$@" 2>&1 | tee "$log_dir/terminal.log"
result=${PIPESTATUS[0]}
set -e
exit "$result"
