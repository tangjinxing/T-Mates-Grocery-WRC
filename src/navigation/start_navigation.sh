#!/usr/bin/env bash
set -euo pipefail

# One-click start: camera + RGB-D + pose API + agent (A) + bridge (B) + Init/Relocate (C).
# HTTP gateway: GET /navigation/health, POST /navigation/navigate
# Camera HTTP: GET /health, GET /camera/snapshot?camera=head&type=color
# RGB-D HTTP: GET /health, GET /camera/rgbd?camera=left|right
# Pose API: GET /pose/health, POST /pose/prepare

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

if is_running_pid "$AGENT_PID_FILE" || is_running_pid "$BRIDGE_PID_FILE" || is_running_pid "$CAMERA_PID_FILE" || is_running_pid "$RGBD_PID_FILE" || is_running_pid "$POSE_PID_FILE"; then
    echo "[error] navigation stack already running; use stop_navigation.sh first" >&2
    exit 1
fi

for required in "$PARAMS_FILE" "$STATIONS_FILE" "$TARGET_MAPPING_FILE"; do
    if [[ ! -f "$required" ]]; then
        echo "[error] missing config: $required" >&2
        exit 1
    fi
done

if [[ ! -f "$CONDA_SH" ]]; then
    echo "[error] conda.sh not found: $CONDA_SH" >&2
    exit 1
fi
if [[ ! -f "$CAMERA_DIR/camera_snapshot_server.py" ]]; then
    echo "[error] missing camera server: $CAMERA_DIR/camera_snapshot_server.py" >&2
    exit 1
fi
if [[ ! -f "$CAMERA_DIR/camera_rgbd_server.py" ]]; then
    echo "[error] missing RGB-D camera server: $CAMERA_DIR/camera_rgbd_server.py" >&2
    exit 1
fi
if [[ ! -f "$POSE_DIR/app.py" ]]; then
    echo "[error] missing pose API: $POSE_DIR/app.py" >&2
    exit 1
fi

echo "=== [0] start head camera HTTP bridge ==="
(
    set +u
    # shellcheck source=/dev/null
    source "$CONDA_SH"
    conda activate "$CONDA_ENV"
    set -u
    cd "$CAMERA_DIR"
    exec python camera_snapshot_server.py
) >"$LOG_DIR/camera.log" 2>&1 &
echo "$!" >"$CAMERA_PID_FILE"
echo "camera pid $(cat "$CAMERA_PID_FILE"), log: $LOG_DIR/camera.log"

echo "=== wait for camera ${CAMERA_URL}/health ==="
if ! wait_for_http "${CAMERA_URL}/health" 30; then
    echo "[error] camera HTTP bridge not responding on port ${CAMERA_PORT}" >&2
    echo "        see $LOG_DIR/camera.log" >&2
    exit 1
fi
echo "[ok] camera HTTP bridge listening"

echo "=== [0a] start RGB-D camera HTTP bridge ==="
(
    set +u
    # shellcheck source=/dev/null
    source "$CONDA_SH"
    conda activate "$CONDA_ENV"
    set -u
    cd "$CAMERA_DIR"
    exec python camera_rgbd_server.py
) >"$LOG_DIR/camera_rgbd.log" 2>&1 &
echo "$!" >"$RGBD_PID_FILE"
echo "rgbd pid $(cat "$RGBD_PID_FILE"), log: $LOG_DIR/camera_rgbd.log"

echo "=== wait for RGB-D ${RGBD_URL}/health ==="
if ! wait_for_http "${RGBD_URL}/health" 30; then
    echo "[error] RGB-D camera HTTP bridge not responding on port ${RGBD_PORT}" >&2
    echo "        see $LOG_DIR/camera_rgbd.log" >&2
    exit 1
fi
echo "[ok] RGB-D camera HTTP bridge listening"

echo "=== [0b] start capability pose API ==="
(
    set +u
    # shellcheck source=/dev/null
    source "$CONDA_SH"
    conda activate "$CONDA_ENV"
    set -u
    cd "$POSE_DIR"
    exec python app.py
) >"$LOG_DIR/pose.log" 2>&1 &
echo "$!" >"$POSE_PID_FILE"
echo "pose pid $(cat "$POSE_PID_FILE"), log: $LOG_DIR/pose.log"

echo "=== wait for pose ${POSE_URL}/pose/health ==="
pose_ready=0
for _ in $(seq 1 90); do
    pose_status="$(fetch_pose_health_status)"
    if [[ "$pose_status" == "READY" ]]; then
        pose_ready=1
        break
    fi
    sleep 1
done
if [[ "$pose_ready" -ne 1 ]]; then
    echo "[error] pose API not READY on port ${POSE_PORT}" >&2
    echo "        see $LOG_DIR/pose.log" >&2
    exit 1
fi
echo "[ok] pose API READY"

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
echo "Camera health: curl --noproxy '*' ${CAMERA_URL}/health"
echo "RGB-D health : curl --noproxy '*' ${RGBD_URL}/health"
echo "RGB-D capture: curl --noproxy '*' '${RGBD_URL}/camera/rgbd?camera=right'"
echo "Pose health  : curl --noproxy '*' ${POSE_URL}/pose/health"
echo "Stop        : bash $SCRIPT_DIR/stop_navigation.sh"
echo "Logs        : $LOG_DIR/agent.log , $LOG_DIR/bridge.log , $LOG_DIR/camera.log , $LOG_DIR/camera_rgbd.log , $LOG_DIR/pose.log"

final_status="$(fetch_health_status)"
echo "Current health: ${final_status:-unknown}"
