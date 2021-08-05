#!/usr/bin/env bash
# helper script to connect to ch340 device
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"

# resolve baudrate
baudrate="9600"
if which jq 1>/dev/null; then
    conf_baudrate=$(jq --raw-output --join-output '.target_overrides["*"]["platform.stdio-baud-rate"]' "$SCRIPT_DIR/mbed_app.json")
    if [[ "$conf_baudrate" =~ ^[0-9]+$ ]]; then
        baudrate=$conf_baudrate
    fi
fi

# ch340 filter and settings
serial_filter="vid=1A86&&pid=7523"
eol_mode="crlf"

# run console
# (note: https://github.com/vznncv/vznncv-miniterm tool should be installed)
exec "vznncv-miniterm" --filter "$serial_filter" --baudrate "$baudrate" --eol "$eol_mode"
