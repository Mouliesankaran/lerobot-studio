#!/usr/bin/env bash
# ==============================================================================
# SO-101 Robotics Hardware Bridge Launcher
# Starts the ROS 2 Humble + Tornado Bridge Node
# ==============================================================================

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WORKSPACE_DIR" || exit 1

if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi

if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
    source "$WORKSPACE_DIR/install/setup.bash"
fi

echo "================================================================="
echo " Starting SO-101 Hardware Bridge on http://localhost:8080"
echo "================================================================="
exec python3 "$WORKSPACE_DIR/src/so101_servo_dynamics/so101_servo_dynamics/web_control_bridge.py"
