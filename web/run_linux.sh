#!/usr/bin/env bash
# ==============================================================================
# SO-101 Robotics Control Studio - Linux 1-Click Launcher
# Cross-Platform Standalone Bridge: Zero ROS 2 Required!
# Works on Ubuntu, Linux Mint, Debian, Arch, Fedora, etc.
# ==============================================================================

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$DIR" || exit 1

echo "====================================================================="
echo "                SO-101 ROBOTICS STUDIO (LINUX)"
echo "====================================================================="
echo ""

# 1. Check Python
if ! command -v python3 &>/dev/null; then
    echo "[ERROR] Python 3 is not installed!"
    echo "Please install python3: sudo apt install python3 python3-pip"
    exit 1
fi

# 2. Check and install dependencies
echo "[*] Checking required Python packages (pyserial, tornado, opencv-python)..."
python3 -c "import serial, tornado" &>/dev/null
if [ $? -ne 0 ]; then
    echo "[*] Installing pyserial and tornado..."
    python3 -m pip install pyserial tornado
fi

# 3. Ensure serial permissions (dialout group)
if ! groups | grep -q "dialout"; then
    echo "[INFO] If serial connection fails, add your user to dialout group: sudo usermod -a -G dialout \$USER"
fi

echo ""
echo "[*] Starting Hardware Bridge on http://localhost:8080..."
echo ""
exec python3 bridge_standalone.py
