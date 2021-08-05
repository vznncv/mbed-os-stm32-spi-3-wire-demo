#!/usr/bin/env bash
#
# Helper script to prepare project for IDE.
#
# It executes the following steps:
# - read and populate `.mbed` file with default values
# - detect custom target usage
# - generates CMakeLists.txt with Mbed CLI 1 or CLI 2
# - generates `openocd_target.cfg` script for OpenOCD 0.11.0 or higher
#
set -e

# Get project dir/name and go to project directory
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" 2>/dev/null 1>&2 && pwd)"
PROJECT_NAME="$(basename "$PROJECT_DIR")"

cd "$PROJECT_DIR"

#
# Utility functions
#
function log() {
    echo "$*" >&2
}
function log_info() {
    log "INFO: $*"
}
function log_error() {
    log "ERROR: $*"
}
function join_lines() {
    printf "%s\n" "$@"
}
function assert_tool_exist() {
    local tool_name="$1"
    if ! which "$tool_name" >/dev/null; then
        log_error "Cannot find required \"${tool_name}\"! Please install it before continuation."
        exit 1
    fi
}

#
# Load base settings
#
APP_MBED_CONF_PATH="$PROJECT_DIR/.mbed"
if [[ -e "$APP_MBED_CONF_PATH" ]]; then
    APP_TOOLCHAIN=$(sed -n 's/TOOLCHAIN=\(.*\)/\1/p' "$APP_MBED_CONF_PATH")
    APP_PROFILE=$(sed -n 's/PROFILE=\(.*\)/\1/p' "$APP_MBED_CONF_PATH")
    APP_TARGET=$(sed -n 's/TARGET=\(.*\)/\1/p' "$APP_MBED_CONF_PATH")
fi
[[ -n "$APP_TOOLCHAIN" ]] || APP_TOOLCHAIN="GCC_ARM"
[[ -n "$APP_PROFILE" ]] || APP_PROFILE="debug"
[[ -n "$APP_TARGET" ]] || APP_TARGET="K64F"
APP_MBED_CLI=1

APP_CUSTOM_TARGET_PATH=""
APP_SOURCE_DIR="src"
APP_INCLUDE_DIR="include"
APP_LIBRARIES=()
MBED_TARGETS_PATH="$PROJECT_DIR/mbed-os/targets/targets.json"
APP_TARGET_FILES=("$MBED_TARGETS_PATH")

# save base settings on exit
function save_config() {
    cat >"$APP_MBED_CONF_PATH" <<EOM
TOOLCHAIN=${APP_TOOLCHAIN}
PROFILE=${APP_PROFILE}
ROOT=.
TARGET=${APP_TARGET}
EOM
}

function on_exit() {
    local exit_code=$?
    if [[ "$exit_code" -eq 0 ]]; then
        save_config
    else
        log_error "Export failed with code $exit_code"
    fi
    return "$exit_code"
}

trap on_exit EXIT

#
# Check base pre-requirements
#
assert_tool_exist "jq"

#
# parse CLI
#
while [[ $# -gt 0 ]]; do
    case "$1" in
    -h | --help)
        log "Prepare project for IDE usage"
        log ""
        log "Usage ./$(basename "$0") [OPTIONS]"
        log ""
        log "Options:"
        log ""
        log "-c,--cli [1|2] - Mbed CLI version to generate CMake config"
        log "-m, --target <TARGET_NAME> - target board name"
        log "-t, --toolchain <TOOLCHAIN_NAME> - toolchain name"
        log "-p, --profile <PROFILE_NAME> - build profile name"
        exit 0
        ;;
    -c | --cli)
        APP_MBED_CLI="$2"
        shift
        shift
        ;;
    -m | --target)
        APP_TARGET="$2"
        shift
        shift
        ;;
    -t | --toolchain)
        APP_TOOLCHAIN="$2"
        shift
        shift
        ;;
    -p | --profile)
        APP_PROFILE="$2"
        shift
        shift
        ;;
    *)
        log_error "Unknown option $1"
        exit 1
        ;;
    esac
done

[[ -n "$APP_MBED_CLI" ]] || (log_error "Mbed CLI version isn't specified" && exit 1)
[[ -n "$APP_TARGET" ]] || (log_error "TARGET isn't specified" && exit 1)
[[ -n "$APP_PROFILE" ]] || (log_error "PROFILE isn't specified" && exit 1)
[[ -n "$APP_TOOLCHAIN" ]] || (log_error "TOOLCHAIN isn't specified" && exit 1)

#
# Detect custom target
#
function targets_has_target() {
    local target_file="$1"
    local target_name="$2"
    local has_target
    has_target=$(jq --raw-output --arg target_name "$target_name" 'has($target_name)' "$target_file")
    if [[ "$has_target" == "true" ]]; then
        return 0
    else
        return 1
    fi
}
function targets_list_hierarchy() {
    local target_name="$1"
    shift
    local target_files=("$@")
    jq --raw-output --arg target_name "$target_name" --slurp '. | add | map_values(.inherits | first) | . as $base_hierarchy |  ($target_name | while(.; $base_hierarchy[.]))' "${target_files[@]}"
}

if ! targets_has_target "$MBED_TARGETS_PATH" "${APP_TARGET}"; then
    # Search custom_target.json
    APP_CUSTOM_TARGET_PATH="${PROJECT_DIR}/TARGET_${APP_TARGET}/custom_targets.json"
    APP_TARGET_FILES+=("$APP_CUSTOM_TARGET_PATH")
    if [[ ! -e "$APP_CUSTOM_TARGET_PATH" ]]; then
        log_error "Cannot find custom target file \"${APP_CUSTOM_TARGET_PATH}\""
        exit 1
    fi
    if ! targets_has_target "${APP_CUSTOM_TARGET_PATH}" "${APP_TARGET}"; then
        log_error "Custom target file \"${APP_CUSTOM_TARGET_PATH}\" doesn't have target \"${APP_TARGET}\""
        exit 1
    fi
fi

#
# gather libraries
#
while read -r filename; do
    lib_name=$(sed -nE 's/^(.*)\.lib$/\1/p' <<<"$filename")
    if [[ -z "$lib_name" || "$lib_name" == "mbed-os" ]]; then
        continue
    fi
    lib_target_name=$(sed -nE 's/^TARGET_(.*)$/\1/p' <<<"$lib_name")
    echo ">$lib_target_name< >$APP_TARGET<"
    if [[ -n "$lib_target_name" && "$lib_target_name" != "$APP_TARGET" ]]; then
        continue
    fi
    APP_LIBRARIES+=("$lib_name")
done <<<"$(ls "$PROJECT_DIR")"

#
# Log configuration
#

log_info '=================================================================='
log_info "Mbed CLI version: ${APP_MBED_CLI}"
log_info "target: ${APP_TARGET}"
log_info "toolchain: ${APP_TOOLCHAIN}"
log_info "profile: ${APP_PROFILE}"
log_info "libraries: ${APP_LIBRARIES[*]}"
if [[ -n "$APP_CUSTOM_TARGET_PATH" ]]; then
    log_info "custom target file: ${APP_CUSTOM_TARGET_PATH}"
fi
log_info '=================================================================='

# export project
if [[ "$APP_MBED_CLI" == "1" ]]; then
    #
    # prepare project with Mbed CLI 1
    #
    assert_tool_exist "mbed-cli"
    APP_MBED_CLI_1_CUSTOM_TARGET_PATH="${PROJECT_DIR}/custom_targets.json"
    if [[ -n "$APP_CUSTOM_TARGET_PATH" ]]; then
        cp "$APP_CUSTOM_TARGET_PATH" "$APP_MBED_CLI_1_CUSTOM_TARGET_PATH"
    fi

    export_command=("mbed-cli" "export" "--ide" "cmake_gcc_arm" "--profile" "$APP_PROFILE" "--target" "$APP_TARGET")
    log_info "Run command: ${export_command[*]}"
    "${export_command[@]}"

    if [[ -n "$APP_CUSTOM_TARGET_PATH" ]]; then
        rm "$APP_MBED_CLI_1_CUSTOM_TARGET_PATH"
    fi

elif [[ "$APP_MBED_CLI" == "2" ]]; then
    #
    # prepare project with Mbed CLI 2
    #
    assert_tool_exist "mbed-tools"
    APP_MBED_CLI_2_CMAKELISTS_PATH="$PROJECT_DIR/CMakeLists.txt"
    APP_MBED_CLI_2_CMAKELISTS_LINES=(
        "# Project \"${PROJECT_NAME}\""
        "cmake_minimum_required(VERSION 3.19.0)"
        "set(MBED_PATH \${CMAKE_CURRENT_SOURCE_DIR}/mbed-os CACHE INTERNAL \"\")"
        "set(MBED_CONFIG_PATH \${CMAKE_CURRENT_SOURCE_DIR} CACHE INTERNAL \"\")"
        "set(APP_TARGET ${PROJECT_NAME})"
        ""
        "# common mbed configuration"
        "include(\${MBED_PATH}/tools/cmake/app.cmake)"
        ""
        "# application target"
        "project(\${APP_TARGET})"
        "add_executable(\${APP_TARGET})"
        "# add application sources"
        "target_include_directories(\${APP_TARGET} PRIVATE \"${APP_INCLUDE_DIR}\")"
        "file(GLOB_RECURSE APP_SOURCES CONFIGURE_DEPENDS \"${APP_SOURCE_DIR}/*.cpp\" \"${APP_SOURCE_DIR}/*.c\")"
        "target_sources(\${APP_TARGET} PRIVATE \${APP_SOURCES})"
        ""
        "# add mbed-os"
        "add_subdirectory(\${MBED_PATH})"
        "target_link_libraries(\${APP_TARGET} mbed-os)"
        "target_link_libraries(\${APP_TARGET} mbed-events)"
        ""
    )

    #    APP_MBED_CLI_2_CMAKELISTS_LINES+=(
    #        "string(TOLOWER \${CMAKE_BUILD_TYPE} APP_LOWERCASE_BUILD_TYPE)"
    #        "if(APP_LOWERCASE_BUILD_TYPE STREQUAL \"debug\")"
    #        "    # set extra debug options"
    #        "    list(APPEND profile_debug_options \"-O0\" \"-Werror=return-type\")"
    #        "    target_compile_options(mbed-core INTERFACE $<$<COMPILE_LANGUAGE:C>:\${profile_debug_options}>)"
    #        "    target_compile_options(mbed-core INTERFACE $<$<COMPILE_LANGUAGE:CXX>:\${profile_debug_options}>)"
    #        "endif()"
    #    )
    APP_MBED_CLI_2_CMAKELISTS_LINES+=(
        "# custom targets/libraries"
    )
    for lib_name in "${APP_LIBRARIES[@]}"; do
        APP_MBED_CLI_2_CMAKELISTS_LINES+=("add_subdirectory(\${CMAKE_CURRENT_SOURCE_DIR}/${lib_name})")
        lib_file_info="$PROJECT_DIR/$lib_name/mbed_lib.json"
        if [[ ! -e "$lib_file_info" ]]; then
            log_error "Cannot find library configuration file $lib_file_info!"
            exit 1
        fi
        if [[ "$lib_name" == "TARGET_${APP_TARGET}" ]]; then
            cmake_lib_name="mbed-$(tr '[A-Z_]' '[a-z-]' <<<"$APP_TARGET")"
        else
            cmake_lib_name=$(jq --raw-output '.name' "$lib_file_info")
            if [[ "$cmake_lib_name" == "null" ]]; then
                cmake_lib_name="$lib_name"
            fi
            cmake_lib_name="mbed-$(tr '[A-Z_]' '[a-z-]' <<<"$cmake_lib_name")"
        fi
        APP_MBED_CLI_2_CMAKELISTS_LINES+=("target_link_libraries(\${APP_TARGET} ${cmake_lib_name})")
    done
    APP_MBED_CLI_2_CMAKELISTS_LINES+=("")

    APP_MBED_CLI_2_CMAKELISTS_LINES+=(
        "mbed_set_post_build(\${APP_TARGET})"
        ""
        "option(VERBOSE_BUILD \"Have a verbose build process\")"
        "if(VERBOSE_BUILD)"
        "    set(CMAKE_VERBOSE_MAKEFILE ON)"
        "endif()"
    )
    log_info "Update CMakeLists.txt"
    join_lines "${APP_MBED_CLI_2_CMAKELISTS_LINES[@]}" >"CMakeLists.txt"

    export_command=("mbed-tools" "configure" "--toolchain" "${APP_TOOLCHAIN}" "--profile" "$APP_PROFILE" "--mbed-target" "$APP_TARGET" "--output-dir" "$PROJECT_DIR")
    if [[ -n "$APP_CUSTOM_TARGET_PATH" ]]; then
        export_command+=("--custom-targets-json" "$APP_CUSTOM_TARGET_PATH")
    fi
    log_info "Run command: ${export_command[*]}"
    "${export_command[@]}"

else
    log_error "Unknown Mbed CLI: $APP_MBED_CLI"
    exit 1
fi

#
# populate "openocd_target.cfg" for openocd 0.11 or higher
#
APP_TARGET_STM_FAMILY=""
while read -r target_name; do
    APP_TARGET_STM_FAMILY=$(sed -nE 's/^.*STM32([A-Z0-9]{2}).*$/\1/p' <<<"$target_name" | tr '[:upper:]' '[:lower:]')
    if [[ -n "$APP_TARGET_STM_FAMILY" ]]; then
        break
    fi
done <<<"$(targets_list_hierarchy "$APP_TARGET" "${APP_TARGET_FILES[@]}")"

if [[ -n "$APP_TARGET_STM_FAMILY" ]]; then
    log_info '=================================================================='
    APP_OPENOCD_CONF_PATH="$PROJECT_DIR/openocd_target.cfg"
    log_info "Generate/update OpenOCD configuration \"openocd_target.cfg\""

    # assume that non-NUCLEO/DISCO boards have no physical reset
    if [[ "$APP_TARGET" =~ (NUCLEO|DISCO) ]]; then
        APP_OPENOCD_HAS_RESET_PIN=1
    else
        APP_OPENOCD_HAS_RESET_PIN=0
    fi

    APP_OPENOCD_CONF=(
        "# OpenOCD board configuration"
        "# note: OpenOCD 0.11 or higher is required"
        ""
        "source [find interface/stlink.cfg]"
        ""
        "transport select hla_swd"
        ""
        "source [find target/stm32${APP_TARGET_STM_FAMILY}x.cfg]"
        ""
        "# reset config (it depends on a board and debugger connection)"
        "# 1. for a board with a physical \"reset\" pin for debugger (like STM boards with embedded debugger)"
        "$([[ "$APP_OPENOCD_HAS_RESET_PIN" -ne 0 ]] || echo -n "# ")reset_config srst_only"
        "# 2. for board without a physical \"reset\" pin for debugger (use this option for boards like bluepill/blackpill)"
        "$([[ "$APP_OPENOCD_HAS_RESET_PIN" -eq 0 ]] || echo -n "# ")reset_config none separate"
    )
    join_lines "${APP_OPENOCD_CONF[@]}" >"$APP_OPENOCD_CONF_PATH"
fi

log_info "Create local .gdbinit file"
GDB_INIT_FILE="$PROJECT_DIR/.gdbinit"
cat <<EOT >>"$GDB_INIT_FILE"
# CLion and gdbinit usage: https://www.jetbrains.com/help/clion/configuring-debugger-options.html#gdbinit-lldbinit

# disable memory protection to allow register access with SVD files
set mem inaccessible-by-default off
EOT

log_info "Complete!"
