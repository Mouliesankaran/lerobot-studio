#!/usr/bin/env bash
# ==============================================================================
# SO-101 Robotics Studio - Ubuntu Dedicated Launcher (22.04 & 24.04 Compatible)
# ==============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "================================================================="
echo "   SO-101 Robotics Studio - Ubuntu Dedicated Launcher"
echo "================================================================="

# 1. Check user group permissions for USB serial and camera
NEED_GROUP_FIX=0
if ! groups "$USER" | grep -q "\bdialout\b"; then
    echo "[!] User '$USER' is not in the 'dialout' group (needed for /dev/ttyACM*)."
    NEED_GROUP_FIX=1
fi
if ! groups "$USER" | grep -q "\bvideo\b"; then
    echo "[!] User '$USER' is not in the 'video' group (needed for /dev/video*)."
    NEED_GROUP_FIX=1
fi
if [ "$NEED_GROUP_FIX" -eq 1 ]; then
    echo "[HINT] Run once: sudo usermod -aG dialout,video $USER (then log out/in)"
fi

# 2. Check ffmpeg installation (for 65+ FPS zero-latency camera streaming)
if ! command -v ffmpeg &> /dev/null; then
    echo "[!] ffmpeg is not installed. Installing ffmpeg..."
    sudo apt update && sudo apt install -y ffmpeg
fi

# 3. Conda Environment Activation
ENV_NAME="so101_studio"
CONDA_BIN=""

if command -v conda &> /dev/null; then
    CONDA_BIN="conda"
elif [ -f "$HOME/miniforge3/bin/conda" ]; then
    CONDA_BIN="$HOME/miniforge3/bin/conda"
elif [ -f "$HOME/miniconda3/bin/conda" ]; then
    CONDA_BIN="$HOME/miniconda3/bin/conda"
elif [ -f "$HOME/anaconda3/bin/conda" ]; then
    CONDA_BIN="$HOME/anaconda3/bin/conda"
fi

if [ -n "$CONDA_BIN" ]; then
    eval "$("$CONDA_BIN" shell.bash hook)"
    if ! conda info --envs | grep -q "^$ENV_NAME\s"; then
        echo "[*] Creating Conda environment '$ENV_NAME' (Python 3.10)..."
        "$CONDA_BIN" env create -f "$SCRIPT_DIR/environment.yml"
    fi
    echo "[*] Activating Conda environment '$ENV_NAME'..."
    conda activate "$ENV_NAME"
    PYTHON_EXE="python3"
else
    echo "[*] Conda not found in PATH. Using Python virtual environment..."
    VENV_DIR="$SCRIPT_DIR/.venv"
    if [ ! -d "$VENV_DIR" ]; then
        echo "[*] Creating virtual environment at $VENV_DIR..."
        python3 -m venv "$VENV_DIR"
        "$VENV_DIR/bin/pip" install --upgrade pip
        "$VENV_DIR/bin/pip" install -r "$SCRIPT_DIR/requirements.txt"
    fi
    PYTHON_EXE="$VENV_DIR/bin/python3"
fi

echo "[*] Starting SO-101 Hardware Bridge on http://localhost:8080..."
exec "$PYTHON_EXE" "$SCRIPT_DIR/bridge_standalone.py"
