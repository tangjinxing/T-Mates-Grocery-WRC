#!/usr/bin/env bash
set -euo pipefail

# Stop camera + pose API + agent + bridge started by start_navigation.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=nav_common.sh
source "$SCRIPT_DIR/nav_common.sh"

stop_pid_file() {
    local name="$1"
    local pid_file="$2"
    if ! [[ -f "$pid_file" ]]; then
        echo "[skip] $name: no pid file"
        return 0
    fi
    local pid
    pid="$(cat "$pid_file")"
    if kill -0 "$pid" 2>/dev/null; then
        echo "[stop] $name (pid $pid)"
        kill "$pid" 2>/dev/null || true
        for _ in $(seq 1 10); do
            kill -0 "$pid" 2>/dev/null || break
            sleep 0.5
        done
        if kill -0 "$pid" 2>/dev/null; then
            echo "[kill] $name (pid $pid)"
            kill -9 "$pid" 2>/dev/null || true
        fi
    else
        echo "[skip] $name: pid $pid not running"
    fi
    rm -f "$pid_file"
}

echo "=== stop WRC navigation stack ==="

stop_pid_file "bridge launch" "$BRIDGE_PID_FILE"
stop_pid_file "woosh agent" "$AGENT_PID_FILE"
stop_pid_file "camera snapshot" "$CAMERA_PID_FILE"
stop_pid_file "camera rgbd" "$RGBD_PID_FILE"
stop_pid_file "pose API" "$POSE_PID_FILE"

# Clean up orphaned nodes if launch did not exit cleanly.
pkill -f 'retail_nav_bridge/retail_nav_bridge' 2>/dev/null || true
pkill -f 'retail_nav_bridge/retail_nav_http_gateway' 2>/dev/null || true
pkill -f 'woosh_robot_agent/agent' 2>/dev/null || true
pkill -f 'camera_snapshot_server.py' 2>/dev/null || true
pkill -f 'camera_rgbd_server.py' 2>/dev/null || true
pkill -f '/capability_api/app.py' 2>/dev/null || true

if ss -ltn 2>/dev/null | grep -q ":${HTTP_PORT} "; then
    echo "[warn] port ${HTTP_PORT} still listening; check remaining processes"
else
    echo "[ok] port ${HTTP_PORT} released"
fi

if ss -ltn 2>/dev/null | grep -q ":${CAMERA_PORT} "; then
    echo "[warn] port ${CAMERA_PORT} still listening; check remaining camera processes"
else
    echo "[ok] port ${CAMERA_PORT} released"
fi

if ss -ltn 2>/dev/null | grep -q ":${RGBD_PORT} "; then
    echo "[warn] port ${RGBD_PORT} still listening; check remaining RGB-D camera processes"
else
    echo "[ok] port ${RGBD_PORT} released"
fi

if ss -ltn 2>/dev/null | grep -q ":${POSE_PORT} "; then
    echo "[warn] port ${POSE_PORT} still listening; check remaining pose API processes"
else
    echo "[ok] port ${POSE_PORT} released"
fi

echo "=== done ==="
