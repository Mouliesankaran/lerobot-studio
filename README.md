# SO-100 / SO-101 Robotics & Vision Control Suite

An advanced, real-time robotics dashboard, teleoperation platform, and characterization suite for the **SO-100** and **SO-101** 6-DOF robotic arms equipped with **Feetech STS3215 serial bus servos (1 Mbps)**, multi-camera computer vision, and autonomous AI model deployment.

---

## 🏛️ System Architecture

```text
                      ┌──────────────────────────────────────────────┐
                      │        Modern Web Browser (Chrome / Edge)    │
                      │           http://localhost:8080              │
                      │  - Light Mode & Dark Mode Responsive Design  │
                      │  - High-Speed WebSockets & REST Dashboard    │
                      └──────────────────────┬───────────────────────┘
                                             │
                        HTTP REST / WebSocket Telemetry (30 - 50 Hz)
                                             │
                                             ▼
                      ┌──────────────────────────────────────────────┐
                      │       SO-101 Python Standalone Bridge        │
                      │            (bridge_standalone.py)            │
                      │                                              │
                      │  ┌────────────────────────────────────────┐  │
                      │  │  Tornado Web Server & WS Engine (8080) │  │
                      │  │  - Universal CORS & Private Network    │  │
                      │  │  - Real-time Multi-Client Broadcast    │  │
                      │  └───────────────────┬────────────────────┘  │
                      │                      │                       │
                      │  ┌───────────────────┴────────────────────┐  │
                      │  │       Hardware Drivers & Backends      │  │
                      │  │  - StandaloneSTS3215Driver (UART 1M)   │  │
                      │  │  - LeaderFollowerManager (50 Hz)       │  │
                      │  │  - CameraManager (V4L2 / RealSense)    │  │
                      │  │  - AIDeploymentEngine (ACT / Diffusion)│  │
                      │  └───────────────────┬────────────────────┘  │
                      └──────────────────────┼───────────────────────┘
                                             │
                        Dual USB Serial & Camera Video Buses
                                             │
                  ┌───────────────────────────┴───────────────────────────┐
                  ▼                                                       ▼
   ┌──────────────────────────────┐                       ┌──────────────────────────────┐
   │   Follower Arm (Active)      │                       │     Leader Arm (Compliant)   │
   │   - Port: /dev/ttyACM0       │                       │   - Port: /dev/ttyACM1       │
   │   - 6x Feetech STS3215       │                       │   - 6x Feetech STS3215       │
   │   - Torque ON (Synchronous)  │                       │   - Torque OFF (Hand Guided) │
   └──────────────────────────────┘                       └──────────────────────────────┘
                  │                                                       │
                  └───────────────────────────┬───────────────────────────┘
                                              ▼
                        ┌──────────────────────────────────────────┐
                        │        Multi-Camera Vision Feeds         │
                        │   - Top Workspace Cam (/dev/video0)      │
                        │   - Wrist Gripper Cam (Eye-in-Hand)      │
                        │   - RealSense / Orbbec RGB + Depth       │
                        └──────────────────────────────────────────┘
```

---

## 🚀 Key Modules & Capabilities

### 1. 🦾 Leader-Follower Teleoperation & LeRobot Episode Recorder
- **Zero-Latency Bilateral Teleoperation**: Low-latency (50 Hz, `<15 ms` round-trip) leader-to-follower mirroring inspired by the Piper robot SDK.
- **Compliant Hand Guiding (Torque OFF)**: Leader arm servos have torque disabled automatically and continuously so the human operator can guide the arm effortlessly with zero resistance.
- **Integrated Camera Studio**: Live dual-camera streams (Workspace Top Cam + Wrist Eye-in-Hand Cam) situated directly above the recording controls with 1-click connect/disconnect.
- **Hugging Face LeRobot Dataset Format**: Record multi-modal demonstration episodes with synchronized actions (leader joint angles), observation states (follower joint angles), and camera snapshots saved into `data/lerobot_dataset/{task}/episode_{XXXX}/` (`trajectory.csv` + `meta.json`).
- **Instant Replay Verification**: Replay any recorded trajectory directly on the physical follower robot with configurable playback speeds (0.5x, 1.0x, 2.0x).
- **Simulation / Virtual Arm Mode**: Test the full bilateral mirroring and demonstration recording pipeline even when only one physical arm is plugged in.

### 2. 🧠 Autonomous AI Model Deployment & Evaluation (AI Deploy)
- **Policy Checkpoint Loader**: Deploy trained imitation learning and reinforcement learning policies (ACT Transformer, Diffusion Policy, PyTorch `.pt`, and JAX `.msgpack`).
- **Operator Safety Interlock**: 60-second timed hardware authorization interlock; automatically freezes motors if the operator does not explicitly approve workspace clearance.
- **Active Camera Observations**: Live streaming of Workspace and Wrist camera feeds feeding policy inference.
- **Live 6-DOF Joint Dynamics**: Real-time target vs actual joint tracking with sub-degree error calculations and real-time execution console.

### 3. ⚡ Hardware Setup & Launch Guide (Setup)
- **5-Step Local Onboarding**: Line-by-line guide for USB permissions, virtual environment activation, bridge launching, and direct package download (`so101_studio_desktop.zip`).
- **Complete System Reset**: Safe clean uninstall scripts to wipe conda environments and temporary build artifacts when needed.

### 4. 🎮 Comprehensive Arm Control & Single-Arm Teleoperation
- **Direct Joint Control**: Live sliders and numerical targeting for all 6 Feetech STS3215 servos with individual torque toggles, zero pose, and home calibration poses.
- **Teleoperation Modes**:
  - ⌨️ **Keyboard Teleoperator**: Hotkey-driven joint jogging and step sizing (1.0°, 2.0°, 5.0°).
  - 🎮 **HTML5 Gamepad API**: Dual-analog stick and D-pad teleoperation with deadband filtering.
  - 🕹️ **On-Screen Virtual Jogger**: Responsive mobile and touch-friendly D-Pad controls.

### 5. 📊 Real-Time Dynamics Monitoring & CSV Recording
- High-frequency 30 Hz observer tracking position error ($\Delta$), bus voltage (V), temperature (°C), and raw encoder ticks.
- Built-in trajectory recorder exporting time-indexed `.csv` datasets for kinematic modeling and playback.

### 6. 📐 Official LeRobot Calibration Wizard
- 4-step guided calibration (Torque disable → Middle pose recording → Dynamic limit detection → Direct export to `so101_calibration.json`).

### 7. 📷 Multi-Camera Vision Studio
- Thread-safe camera manager supporting Intel RealSense (RGB + Depth), UVC USB webcams, and synthetic test streams.
- Live brightness, contrast, and exposure tuning with snapshot downloads.

---

## 📁 Repository Structure

```text
servo_test/
├── bridge_standalone.py                 # Primary Tornado HTTP & WebSocket Hardware Bridge
├── camera_bridge.py                     # Standalone Bridge Mirror (Launch Script Compatible)
├── camera_backend.py                    # Multi-camera V4L2 & RealSense capture manager
├── leader_follower_manager.py           # 50 Hz Leader-Follower mirroring & LeRobot dataset engine
├── ai_deployment_backend.py             # Autonomous AI policy inference and safety engine
├── environment.yml                      # Conda environment definition (so101_studio)
├── requirements.txt                     # Python package dependencies
├── run_ubuntu.sh                        # 1-Click automated Linux launcher
├── run_windows.bat                      # Windows launcher batch script
├── so101_calibration.json               # Calibrated STS3215 servo offsets and limits
├── home_position.txt                    # Calibrated reference home pose
├── zero_position.txt                    # Baseline zero coordinates
├── README.md                            # Comprehensive system documentation
│
├── web/                                 # Frontend Web Studio (Static Single-Page Application)
│   ├── index.html                       # Responsive HTML5 Interface with Light/Dark Theme
│   ├── styles.css                       # Core dashboard stylesheets
│   ├── setup_page.css / .js             # Hardware launch guide & setup components
│   ├── ai_deploy.css / .js              # Autonomous AI deployment layout & telemetry
│   ├── teleop_leader.css / .js          # Leader-Follower teleoperation, dual cameras & dataset recorder
│   ├── so101_studio_desktop.zip         # Standalone offline desktop distribution archive
│   ├── logo.png                         # iHub Robotics branding logo
│   └── robot_preview.png                # Hardware preview rendering
│
├── data/
│   └── lerobot_dataset/                 # Recorded Hugging Face LeRobot demonstration episodes
│       └── {task_name}/
│           └── episode_{XXXX}/
│               ├── trajectory.csv       # Action & observation state trajectories
│               └── meta.json            # Episode metadata, FPS, frame count, duration
│
└── uploads/                             # Uploaded checkpoints and model weights
    └── checkpoints/                     # AI policy checkpoints (.pt, .msgpack)
```

---

## 🛠️ Quick Start & Installation

### Step 1: Configure USB Permissions (Linux)
```bash
# Grant access to serial USB ports and camera devices (Run once)
sudo usermod -aG dialout,video $USER
```
*(Log out and log back in, or restart your terminal session for permissions to take effect).*

### Step 2: Set Up Python Environment
Using Conda:
```bash
conda env create -f environment.yml
conda activate so101_studio
```

Or using standard `pip`:
```bash
pip install -r requirements.txt
```

### Step 3: Launch Hardware Bridge & Studio
Run the automated launcher:
```bash
chmod +x run_ubuntu.sh
./run_ubuntu.sh
```

Or run the Python bridge directly:
```bash
python3 bridge_standalone.py --port /dev/ttyACM0 --web-port 8080
```

### Step 4: Open in Web Browser
Open **`http://localhost:8080`** in Google Chrome or Microsoft Edge.

---

## 🦾 Leader-Follower Teleoperation & LeRobot Dataset Workflow

Inspired by the Piper robot SDK and Hugging Face LeRobot, this workflow allows an operator to collect physical demonstrations by hand-guiding the Leader arm while the Follower arm executes identical movements:

### 1. Hardware Connections
- **Follower Arm USB**: Plug into `/dev/ttyACM0` (Torque ACTIVE).
- **Leader Arm USB**: Plug into `/dev/ttyACM1` or `/dev/ttyUSB0` (Torque OFF for free hand guidance).
- **Cameras**: Plug in Workspace Top Camera and Wrist Camera (e.g. `/dev/video0`).

### 2. Open Leader Teleop Dashboard
- Click **`🦾 Leader Teleop`** directly in the left sidebar menu.

### 3. Configure Ports & Start Mirroring
- **Leader Arm Port**: Select `/dev/ttyACM1` (or select `simulation` to test with one arm).
- **Follower Arm Port**: Select `/dev/ttyACM0`.
- Click **`⚡ Leader Torque: OFF (Hand Guiding)`** to ensure the leader arm is compliant and freely movable by hand.
- Click **`▶ Connect Camera`** on the top right to start the synchronized video feed.
- Click **`▶ Start Mirroring`** to begin 50 Hz real-time bilateral tracking!

### 4. Record Demonstration Episodes
1. Enter your task goal (e.g. `Pick cube and place in target bowl`).
2. Click **`🔴 Start Episode`** to begin logging timestamps, actions, and observations.
3. Guide the Leader arm by hand to pick the cube with the gripper, place it in the target location, and return to the ready pose.
4. Click **`💾 Stop & Save`**: Episode is instantly saved to `data/lerobot_dataset/{task}/episode_{XXXX}/`.
5. If an error occurred during the trial, click **`🗑 Discard`** to discard the attempt without polluting your training dataset.

### 5. Verify Demonstrations (Replay)
- In the **Recorded Dataset Episodes** table, click **`▶ Replay`** on any saved episode to watch the physical follower robot autonomously reproduce the recorded motion!

---

## 🧠 Autonomous AI Policy Deployment Workflow

1. Navigate to the **AI Deploy** tab in the sidebar.
2. Select or import a trained model checkpoint (`.pt`, `.msgpack`).
3. Verify that the workspace is clear of obstacles.
4. Click **Approve (60 s)** to arm the safety interlock.
5. Click **Start Policy** to execute vision-based autonomous trajectories on the physical robot.
6. In case of unexpected behavior, hit **EMERGENCY STOP** (or press the global header E-STOP button) to immediately freeze all servos and disable torque.

---

## 🎨 Theme Customization

SO-101 Studio includes dynamic high-contrast themes:
- **Dark Mode**: High-contrast navy palette designed for laboratory environments (`#080d1a`, `#0d1527`, `#38bdf8`).
- **Light Mode**: Clean daylight palette (`#ffffff`, `#f8fafc`, `#0f172a`, `#0284c7`) with zero dark artifacts or unreadable contrast.

Toggle between themes anytime using the sun/moon button in the top navigation bar.

---

## 📄 License

MIT License. Developed for open-source robotics research, autonomous manipulation, and compliant bilateral teleoperation.
