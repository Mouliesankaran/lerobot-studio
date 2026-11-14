@echo off
REM ==============================================================================
REM SO-101 Robotics Control Studio - Windows 1-Click Launcher
REM Zero ROS 2 Required! Works on any Windows 10/11 machine.
REM ==============================================================================
title SO-101 Robotics Studio Launcher
color 0b

echo =====================================================================
echo                SO-101 ROBOTICS STUDIO (WINDOWS)
echo =====================================================================
echo.

REM 1. Check Python installation
where python >nul 2>nul
if %ERRORLEVEL% NEQ 0 (
    echo [ERROR] Python is not installed or not in PATH!
    echo Please install Python 3.10+ from https://www.python.org/downloads/
    echo (Make sure to check "Add Python to PATH" during installation)
    pause
    exit /b 1
)

echo [*] Checking required Python dependencies...
python -c "import serial, tornado" >nul 2>nul
if %ERRORLEVEL% NEQ 0 (
    echo [*] Installing pyserial and tornado...
    python -m pip install pyserial tornado
)

echo.
echo [*] Starting Hardware Bridge and opening Web Studio...
echo.
python bridge_standalone.py

pause
