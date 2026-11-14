SO-101 ROBOTICS CONTROL STUDIO - STANDALONE BRIDGE (Windows / Linux)
=====================================================================
WINDOWS
 1. Install Python 3.10-3.12 from python.org (keep "py launcher" / "Add to PATH" ticked).
 2. Unzip this folder anywhere. Double-click run_windows.bat
    - first run creates a private .venv and installs pyserial, tornado, opencv-python, numpy
    - for Intel RealSense depth:  run_windows.bat --with-realsense
    - for Orbbec depth (experimental): run_windows.bat --with-orbbec
 3. Browser opens http://localhost:8080  (use Chrome or Edge).
 4. Arm not found? Install the CH340/CH343 (or CP210x) USB-serial driver, re-plug, press Connect Robot.
 5. Camera not found? Close Zoom/Teams/other browser tabs using it; Settings > Privacy > Camera must allow
    desktop apps.  RealSense needs a USB 3 port + cable.

LINUX (Ubuntu, Debian, Mint, Fedora, Arch ...)
    chmod +x run_linux.sh && ./run_linux.sh          (add --with-realsense / --with-orbbec if needed)
    One-time permissions:  sudo usermod -aG dialout,video,plugdev $USER   (Arch: uucp instead of dialout)
    then log out and in.  RealSense/Orbbec also need their udev rules (see CROSS_PLATFORM_REPORT.md).

SELF-TEST (bridge running):   python tests/camera_acceptance.py      (needs the repo tests/ folder)
DIAGNOSTICS:                  http://localhost:8080/api/system/info

FILES: bridge_standalone.py + camera_backend.py (server) | index.html, styles.css (UI) |
       requirements-bridge.txt | home/zero_position.txt, so101_calibration.json
