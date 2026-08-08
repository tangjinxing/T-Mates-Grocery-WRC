#!/usr/bin/env bash
set -euo pipefail

# One-click start: agent (A) + bridge (B) + Init/Relocate (C).
# HTTP gateway: GET /navigation/health, POST /navigation/navigate

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=nav_common.sh
source "$SCRIPT_DIR/nav_common.sh"

RESTART=1
SKIP_INIT=0

usage() {
    cat <<EOF
Usage: $(basename "$0") [options]

Options:
  --no-restart   Do not stop existing stack before start
  --skip-init    Start agent/bridge only; skip Init/Relocate
  -h, --help     Show this help

After success:
  curl --noproxy '*' ${HTTP_URL}/navigation/health
  curl --noproxy '*' -X POST ${HTTP_URL}/navigation/navigate \\
    -H 'Content-Type: application/json' \\
    -H 'Idempotency-Key: nav-001' \\
    -d '{"target_id":"mark_0"}'
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --no-restart) RESTART=0; shift ;;
        --skip-init) SKIP_INIT=1; shift ;;
        -h | --help) usage; exit 0 ;;
        *)
            echo "Unknown option: $1" >&2
            usage
            exit 1
            ;;
    esac
done

if [[ "$RESTART" -eq 1 ]]; then
    bash "$SCRIPT_DIR/stop_navigation.sh" || true
fi

ensure_dirs
setup_ros_env

if is_running_pid "$AGENT_PID_FILE" || is_running_pid "$BRIDGE_PID_FILE"; then
    echo "[error] navigation stack already running; use stop_navigation.sh first" >&2
    exit 1
fi

for required in "$PARAMS_FILE" "$STATIONS_FILE" "$TARGET_MAPPING_FILE"; do
    if [[ ! -f "$required" ]]; then
        echo "[error] missing config: $required" >&2
        exit 1
    fi
done

echo "=== [A] start woosh agent ==="
nohup ros2 run woosh_robot_agent agent --ros-args \
    -r __ns:=/woosh_robot \
    -p ip:="${CHASSIS_IP}" \
    >"$LOG_DIR/agent.log" 2>&1 &
echo "$!" >"$AGENT_PID_FILE"
echo "agent pid $(cat "$AGENT_PID_FILE"), log: $LOG_DIR/agent.log"

echo "=== wait for /woosh_robot/agent ==="
agent_ready=0
for _ in $(seq 1 30); do
    if ros2 node list 2>/dev/null | grep -q '/woosh_robot/agent'; then
        agent_ready=1
        break
    fi
    sleep 1
done
if [[ "$agent_ready" -ne 1 ]]; then
    echo "[error] woosh agent did not appear in ros2 node list" >&2
    echo "        see $LOG_DIR/agent.log" >&2
    exit 1
fi
echo "[ok] woosh agent online"

echo "=== [B] start retail_nav_bridge + HTTP gateway ==="
nohup ros2 launch retail_nav_bridge woosh_bridge.launch.py \
    params_file:="$PARAMS_FILE" \
    stations_file:="$STATIONS_FILE" \
    target_mapping_file:="$TARGET_MAPPING_FILE" \
    http_host:=0.0.0.0 \
    http_port:="$HTTP_PORT" \
    >"$LOG_DIR/bridge.log" 2>&1 &
echo "$!" >"$BRIDGE_PID_FILE"
echo "bridge pid $(cat "$BRIDGE_PID_FILE"), log: $LOG_DIR/bridge.log"

echo "=== wait for HTTP ${HTTP_URL}/navigation/health ==="
if ! wait_for_http "${HTTP_URL}/navigation/health" 90; then
    echo "[error] HTTP gateway not responding on port ${HTTP_PORT}" >&2
    echo "        see $LOG_DIR/bridge.log" >&2
    exit 1
fi
echo "[ok] HTTP gateway listening"

health_status="$(fetch_health_status)"
echo "[health] initial status: ${health_status:-unknown}"

if [[ "$SKIP_INIT" -eq 0 ]]; then
    echo "=== [C] Init + Relocate ==="
    if ! timeout 45 ros2 service call /retail_nav/v1/Init retail_nav_msgs/srv/Init "{}"; then
        echo "[warn] Init failed; check chassis state and $LOG_DIR/bridge.log" >&2
    fi
    if ! timeout 45 ros2 service call /retail_nav/v1/Relocate retail_nav_msgs/srv/Relocate "{}"; then
        echo "[warn] Relocate failed; may need HMI recovery or ClearError" >&2
    fi

    echo "=== wait for READY ==="
    ready=0
    for _ in $(seq 1 30); do
        health_status="$(fetch_health_status)"
        echo "[health] ${health_status:-unknown}"
        if [[ "$health_status" == "READY" ]]; then
            ready=1
            break
        fi
        sleep 2
    done
    if [[ "$ready" -ne 1 ]]; then
        echo "[warn] health is not READY yet; HTTP queries still work" >&2
    fi
fi

echo
echo "=== navigation stack started ==="
echo "HTTP health : curl --noproxy '*' ${HTTP_URL}/navigation/health"
echo "HTTP navigate: curl --noproxy '*' -X POST ${HTTP_URL}/navigation/navigate \\"
echo "  -H 'Content-Type: application/json' -H 'Idempotency-Key: nav-001' \\"
echo "  -d '{\"target_id\":\"mark_0\"}'"
echo "Stop        : bash $SCRIPT_DIR/stop_navigation.sh"
echo "Logs        : $LOG_DIR/agent.log , $LOG_DIR/bridge.log"

final_status="$(fetch_health_status)"
echo "Current health: ${final_status:-unknown}"
