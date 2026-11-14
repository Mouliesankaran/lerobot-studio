================================================================================
SO-101 ROBOTICS CONTROL STUDIO - WINDOWS STANDALONE BRIDGE
================================================================================

HOW TO RUN ON WINDOWS:
1. Make sure Python 3.10+ is installed (https://www.python.org/downloads/)
   * Check "Add Python to PATH" during installation.
2. Double-click "run_windows.bat".
3. That's it! The launcher will automatically:
   - Check and install required packages (pyserial, tornado).
   - Auto-detect your robot's COM port (CH340 / CP2102).
   - Launch the local bridge server on http://localhost:8080.
   - Automatically open the Robotics Studio dashboard in your browser.

In the web interface:
- Connection Mode: Keep set to "Local Bridge".
- Click "▶ Connect Robot".
- Full 6-DOF physical arm control, live telemetry, and webcam vision are active!

FILES INCLUDED:
- run_windows.bat        : 1-click Windows launcher
- bridge_standalone.py   : Pure Python STS3215 hardware bridge (zero ROS 2 required)
- home_position.txt      : Calibrated default home pose
- zero_position.txt      : Canonical reference zero pose
- so101_calibration.json : Hardware limits and homing tick offsets
================================================================================
