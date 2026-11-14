@echo off
setlocal EnableDelayedExpansion
REM ==============================================================================
REM SO-101 Robotics Control Studio - Windows 1-click launcher (no ROS 2 needed)
REM Usage: run_windows.bat [--with-realsense] [--with-orbbec] [--test-camera]
REM ==============================================================================
title SO-101 Robotics Studio Launcher
color 0b
cd /d "%~dp0"

echo =====================================================================
echo                SO-101 ROBOTICS STUDIO (WINDOWS)
echo =====================================================================
echo.

set "EXTRA="
set "ARGS="
:parse
if "%~1"=="" goto parsed
if /I "%~1"=="--with-realsense" ( set "EXTRA=!EXTRA! pyrealsense2" ) else if /I "%~1"=="--with-orbbec" ( set "EXTRA=!EXTRA! pyorbbecsdk" ) else ( set "ARGS=!ARGS! %~1" )
shift
goto parse
:parsed

REM 1. Find a REAL Python (the 'python' in WindowsApps is only a Microsoft Store stub)
set "PYCMD="
py -3 -c "import sys" >nul 2>nul && set "PYCMD=py -3"
if not defined PYCMD (
    python -c "import sys; sys.exit(0 if sys.version_info[0]==3 else 1)" >nul 2>nul && set "PYCMD=python"
)
if not defined PYCMD (
    echo [ERROR] Python 3 was not found.
    echo Install Python 3.10-3.12 from https://www.python.org/downloads/
    echo and tick "Add python.exe to PATH" ^(or keep the "py launcher" option on^).
    pause
    exit /b 1
)

REM 2. Private virtual-env: no admin rights, no conflicts with other Python projects
if not exist ".venv\Scripts\python.exe" (
    echo [*] Creating private Python environment ^(.venv^) ...
    %PYCMD% -m venv .venv
    if errorlevel 1 ( echo [ERROR] Could not create virtual environment. & pause & exit /b 1 )
)
set "VPY=%~dp0.venv\Scripts\python.exe"

REM 3. Dependencies
"%VPY%" -c "import serial, tornado, cv2, numpy" >nul 2>nul
if errorlevel 1 (
    echo [*] Installing dependencies ^(pyserial, tornado, opencv-python, numpy^) ...
    "%VPY%" -m pip install --quiet --upgrade pip
    "%VPY%" -m pip install -r requirements-bridge.txt
    if errorlevel 1 ( echo [ERROR] pip install failed - check your internet connection. & pause & exit /b 1 )
)
for %%P in (%EXTRA%) do (
    echo [*] Installing optional camera SDK: %%P ...
    "%VPY%" -m pip install --quiet %%P
    if errorlevel 1 echo [WARN] Could not install %%P for this Python version - that camera will be unavailable.
)

echo.
echo [*] Starting bridge on http://localhost:8080 ...
echo     ^(If the arm is not found: install the CH340/CH343 USB driver, then re-plug.^)
echo.
"%VPY%" bridge_standalone.py%ARGS%

pause
