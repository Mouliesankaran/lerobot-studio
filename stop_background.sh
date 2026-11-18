#!/usr/bin/env bash
# ==============================================================================
# Stop SO-101 Background Bridge Server
# ==============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PID_FILE="$SCRIPT_DIR/.so101_bridge.pid"

STOPPED=0

if [ -f "$PID_FILE" ]; then
    PID=$(cat "$PID_FILE")
    if ps -p "$PID" > /dev/null 2>&1; then
        echo "[*] Stopping SO-101 Bridge (PID: $PID)..."
        kill "$PID" 2>/dev/null || true
        sleep 1
        kill -9 "$PID" 2>/dev/null || true
        STOPPED=1
    fi
    rm -f "$PID_FILE"
fi

# Fallback check on port 8080 or bridge_standalone.py
PIDS=$(pgrep -f "bridge_standalone.py" || true)
if [ -n "$PIDS" ]; then
    echo "[*] Killing remaining bridge processes: $PIDS"
    kill $PIDS 2>/dev/null || true
    sleep 0.5
    kill -9 $PIDS 2>/dev/null || true
    STOPPED=1
fi

if [ "$STOPPED" -eq 1 ]; then
    echo "[OK] SO-101 Localhost server has been stopped."
else
    echo "[i] SO-101 Localhost server was not running."
fi
