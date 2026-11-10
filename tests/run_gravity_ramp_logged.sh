#!/usr/bin/env bash
# Operator-launched, bounded gravity trial with passive CAN capture.
set -euo pipefail
script_dir=$(cd -- "$(dirname -- "$0")" && pwd)
tool="$script_dir/build/ex04_gravity_ramp"
iface=can1
if ! { [[ $# == 0 ]] || { [[ $# == 1 ]] && [[ "$1" == "--30-percent" ]]; }; }; then
    echo "Usage: $0 [--30-percent]" >&2
    echo "Default: 10% / 3 s ramp / 0.5 s hold. --30-percent: 30% / 10 s ramp / 60 s hold." >&2
    exit 2
fi
[[ -x "$tool" ]] || { echo "Missing $tool" >&2; exit 1; }
command -v candump >/dev/null
command -v stdbuf >/dev/null
if pgrep -u "$UID" -f '[b]t-wam-|[e]x04_display_basic_info|[e]x04_gravity_ramp|[l]bc_torque_sign|[l]bc_position_watch|[w]am8_j8_sweep|[e]x01_initialize_wam|[w]am8g_state' >/dev/null; then
    echo "Another WAM program is running or stopped. Exit it before this diagnostic." >&2
    exit 1
fi
mkdir -p "$HOME/sbflash"
log_dir=$(mktemp -d "$HOME/sbflash/gravity-ramp-$(date +%Y%m%d-%H%M%S)-XXXXXX")
export BARRETT_CONFIG_FILE="$HOME/.barrett/default.conf"
for file in default.conf wam8g.conf calibration_data/wam8g/zerocal.conf calibration_data/wam8g/gravitycal.conf; do
    cp -- "$HOME/.barrett/$file" "$log_dir/$(basename -- "$file")"
done
sha256sum "$tool" "$log_dir/"*.conf > "$log_dir/sha256.txt"
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
    echo "CAN recording failed; diagnostic was not started." >&2
    exit 1
fi
echo "Recording CAN, terminal output, calibration and CSV in $log_dir"
printf '%q ' "$tool" "$@" --log "$log_dir/samples.csv" > "$log_dir/command.txt"
printf '\n' >> "$log_dir/command.txt"
set +e
stdbuf -o0 -e0 "$tool" "$@" --log "$log_dir/samples.csv" 2>&1 | tee "$log_dir/terminal.log"
result=${PIPESTATUS[0]}
set -e
exit "$result"
