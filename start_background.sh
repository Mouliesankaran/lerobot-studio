#!/usr/bin/env bash
# ==============================================================================
# Start SO-101 Studio Bridge in the Background (No Terminal Needed)
# ==============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PID_FILE="$SCRIPT_DIR/.so101_bridge.pid"
LOG_FILE="$SCRIPT_DIR/log/bridge_daemon.log"

mkdir -p "$SCRIPT_DIR/log"

# Check if already running
if [ -f "$PID_FILE" ]; then
    PID=$(cat "$PID_FILE")
    if ps -p "$PID" > /dev/null 2>&1; then
        echo "[!] SO-101 Bridge is already running (PID: $PID) on http://localhost:8080"
        exit 0
    else
        rm -f "$PID_FILE"
    fi
fi

# Run launcher in background via nohup
nohup "$SCRIPT_DIR/run_ubuntu.sh" --no-browser > "$LOG_FILE" 2>&1 &
PID=$!
echo "$PID" > "$PID_FILE"

sleep 1.5

if ps -p "$PID" > /dev/null 2>&1; then
    echo "================================================================="
    echo " [OK] SO-101 Localhost Server is now running 24/7 in background!"
    echo " URL     : http://localhost:8080"
    echo " PID     : $PID"
    echo " Log File: $LOG_FILE"
    echo " Stop    : ./stop_background.sh"
    echo "================================================================="
else
    echo "[ERROR] Server failed to start in background. Check $LOG_FILE for details."
    cat "$LOG_FILE" | tail -n 15
    exit 1
fi
