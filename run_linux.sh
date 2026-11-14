#!/usr/bin/env bash
# ==============================================================================
# SO-101 Robotics Control Studio - Linux 1-click launcher (no ROS 2 needed)
# Works on Ubuntu, Debian, Mint, Fedora, Arch, openSUSE ...
# Uses a private virtual-env (.venv) so it also works on distros that block
# system-wide pip (PEP 668: Ubuntu 23.04+, Debian 12+, Fedora 38+).
#
# Usage: ./run_linux.sh [--with-realsense] [--with-orbbec] [--test-camera] [bridge args...]
# ==============================================================================
set -u
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$DIR" || exit 1

echo "====================================================================="
echo "                SO-101 ROBOTICS STUDIO (LINUX)"
echo "====================================================================="

EXTRA_PKGS=()
BRIDGE_ARGS=()
for a in "$@"; do
  case "$a" in
    --with-realsense) EXTRA_PKGS+=("pyrealsense2") ;;
    --with-orbbec)    EXTRA_PKGS+=("pyorbbecsdk") ;;
    *)                BRIDGE_ARGS+=("$a") ;;
  esac
done

PY="$(command -v python3 || command -v python || true)"
if [ -z "$PY" ]; then
  echo "[ERROR] Python 3 not found. Install it:"
  echo "   Debian/Ubuntu: sudo apt install python3 python3-venv python3-pip"
  echo "   Fedora:        sudo dnf install python3 python3-pip"
  echo "   Arch:          sudo pacman -S python python-pip"
  exit 1
fi
"$PY" -c 'import sys; sys.exit(0 if sys.version_info >= (3,8) else 1)' || { echo "[ERROR] Python 3.8+ required."; exit 1; }

# 1. private venv
if [ ! -x ".venv/bin/python" ]; then
  echo "[*] Creating private Python environment (.venv) ..."
  if ! "$PY" -m venv .venv 2>/dev/null; then
    echo "[ERROR] Could not create a venv. On Debian/Ubuntu run: sudo apt install python3-venv"
    exit 1
  fi
fi
VPY="$DIR/.venv/bin/python"

# 2. dependencies (only when something is missing)
if ! "$VPY" -c "import serial, tornado, cv2, numpy" 2>/dev/null; then
  echo "[*] Installing dependencies (pyserial, tornado, opencv-python, numpy) ..."
  "$VPY" -m pip install --quiet --upgrade pip
  "$VPY" -m pip install -r "$DIR/requirements-bridge.txt" || { echo "[ERROR] pip install failed (no internet?)."; exit 1; }
fi
for pkg in "${EXTRA_PKGS[@]:-}"; do
  [ -z "$pkg" ] && continue
  echo "[*] Installing optional camera SDK: $pkg ..."
  "$VPY" -m pip install --quiet "$pkg" || echo "[WARN] Could not install $pkg for this Python/CPU - that camera will be unavailable."
done

# 3. permissions (serial + camera) - hints only, never sudo
if [[ "$(uname -s)" == "Linux" ]]; then
  ME="$(id -un)"
  need=()
  for g in dialout uucp video plugdev; do
    getent group "$g" >/dev/null 2>&1 && ! id -nG "$ME" | tr ' ' '\n' | grep -qx "$g" && need+=("$g")
  done
  if [ ${#need[@]} -gt 0 ]; then
    echo "[INFO] You are not in group(s): ${need[*]}"
    echo "       Serial port needs dialout (Arch: uucp), cameras need video, RealSense needs plugdev."
    echo "       Fix once:  sudo usermod -aG $(IFS=,; echo "${need[*]}") $ME   (then log out and in)"
  fi
fi

echo "[*] Starting bridge on http://localhost:8080 ..."
exec "$VPY" "$DIR/bridge_standalone.py" "${BRIDGE_ARGS[@]:-}"
