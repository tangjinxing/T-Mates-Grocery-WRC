#!/usr/bin/env bash
# Shared configuration for WRC navigation scripts.

NAV_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WRC_ROOT="$(cd "$NAV_ROOT/../.." && pwd)"
CONFIG_DIR="$NAV_ROOT/retail_nav_bridge/config"
LOG_DIR="$NAV_ROOT/logs"
RUN_DIR="$NAV_ROOT/run"

PARAMS_FILE="$CONFIG_DIR/nav_defaults_woosh.yaml"
STATIONS_FILE="$CONFIG_DIR/retail_stations.yaml"
TARGET_MAPPING_FILE="$CONFIG_DIR/product_slot_navigation.yaml"

CHASSIS_IP="${CHASSIS_IP:-169.254.128.2}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
HTTP_PORT="${HTTP_PORT:-8081}"
HTTP_URL="http://127.0.0.1:${HTTP_PORT}"

AGENT_PID_FILE="$RUN_DIR/agent.pid"
BRIDGE_PID_FILE="$RUN_DIR/bridge.pid"

setup_ros_env() {
    # ROS setup.bash references optional vars; disable nounset while sourcing.
    set +u
    # shellcheck source=/dev/null
    source /opt/ros/humble/setup.bash
    # shellcheck source=/dev/null
    source "$WRC_ROOT/install/setup.bash"
    set -u
    export ROS_DOMAIN_ID
}

ensure_dirs() {
    mkdir -p "$LOG_DIR" "$RUN_DIR"
}

curl_noproxy() {
    curl --noproxy '*' "$@"
}

wait_for_http() {
    local url="$1"
    local max_wait="${2:-90}"
    local i
    for ((i = 1; i <= max_wait; i++)); do
        if curl_noproxy -sf -m 2 "$url" >/dev/null 2>&1; then
            return 0
        fi
        sleep 1
    done
    return 1
}

fetch_health_status() {
    curl_noproxy -sf -m 3 "$HTTP_URL/navigation/health" \
        | python3 -c 'import json,sys; print(json.load(sys.stdin).get("status",""))' 2>/dev/null \
        || true
}

is_running_pid() {
    local pid_file="$1"
    [[ -f "$pid_file" ]] || return 1
    local pid
    pid="$(cat "$pid_file")"
    [[ -n "$pid" ]] || return 1
    kill -0 "$pid" 2>/dev/null
}
