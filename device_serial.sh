#!/usr/bin/env bash
#
# Helper script for connection to MCU serial input/output (stdin/stout)
#
set -e
# Get project dir/name and go to project directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" 2>/dev/null 1>&2 && pwd)"

#
# Utility functions
#
function log() {
    echo "$*" >&2
}
function log_error() {
    log "ERROR: $*"
}
function assert_tool() {
    local tool_name="$1"
    local error_message=$2
    if ! which "$tool_name" >/dev/null; then
        log_error "Cannot find tool \"${tool_name}\"! $error_message"
        exit 1
    fi
}
function read_app_param() {
    local jq_filter="$1"
    assert_tool jq "Please install jq to use ./$(basename "$0") script"
    if [[ -f "$SCRIPT_DIR/mbed_app.json" ]]; then
        jq --raw-output --join-output "$jq_filter" "$SCRIPT_DIR/mbed_app.json"
    else
        echo "null"
    fi
}

# Default settings
DEFAULT_BAUDRATE=9600
DEFAULT_EOL_MODE="crlf"
DEFAULT_FILTER=("vid=1A86&&pid=7523" "vid=0483&&pid=374b")

# Serial parameters
baudrate=""
eol_mode=""
filter=()
output_file=""
#
# parse CLI
#
while [[ $# -gt 0 ]]; do
    case "$1" in
    -h | --help)
        log "Connect to project for IDE usage"
        log ""
        log "Usage ./$(basename "$0") [OPTIONS]"
        log ""
        log "Options:"
        log ""
        log "--baudrate <baudrate> - explicit device baudrate (by default script tries to extract value from project settigns)"
        log "--eol <eol_mode> - end of line mode. Default value - 'crlf'"
        log "--filter <filter_expr> - one or more filter expression (see vznncv-miniterm for more details)"
        log "--output-file <file> - output file to save serial output"
        exit 0
        ;;
    --baudrate)
        baudrate="$2"
        shift
        shift
        ;;
    --eol|--eol-mode)
        eol_mode="$2"
        shift
        shift
        ;;
    --filter)
        filter+=("$2")
        shift
        shift
        ;;
    --output-file)
        output_file="$2"
        shift
        shift
        ;;
    *)
        log_error "Unknown option $1"
        exit 1
        ;;
    esac
done

# check tools
assert_tool "vznncv-miniterm" "Please install vznncv-miniterm (https://github.com/vznncv/vznncv-miniterm) to use ./$(basename "$0") script"

# resolve baudrate
if [[ -z "$baudrate" ]]; then
    baudrate=$(read_app_param '.target_overrides["*"]["platform.stdio-baud-rate"]')
    if [[ -z "$baudrate" || "$baudrate" == 'null' ]]; then
        baudrate="$DEFAULT_BAUDRATE"
    fi
fi
# resole eol_mode
if [[ -z "$eol_mode" ]]; then
    eol_mode="$DEFAULT_EOL_MODE"
fi
# resolve filter
if [[ "${#filter[@]}" -eq 0 ]]; then
    filter=("${DEFAULT_FILTER[@]}")
fi

# build args
vznncv_miniterm_args=()
for filter_expr in "${filter[@]}"; do
    vznncv_miniterm_args+=("--filter" "$filter_expr")
done
vznncv_miniterm_args+=("--baudrate" "$baudrate")
vznncv_miniterm_args+=("--eol" "$eol_mode")
if [[ -n "$output_file" ]]; then
    vznncv_miniterm_args+=("--output-file" "$output_file")
fi

exec "vznncv-miniterm" "${vznncv_miniterm_args[@]}"
