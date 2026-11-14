# SO-100 / SO-101 Robotics & Vision Control Suite

An advanced, real-time robotics dashboard and characterization platform for the **SO-100** and **SO-101** 6-DOF robotic arms equipped with **Feetech STS3215 serial bus servos**, integrated with **ROS 2 Humble**, multi-camera vision, and cloud deployment via **GitHub + Render**.

---

## Architecture Overview

```
                      ┌──────────────────────────────────────────────┐
                      │            Render Cloud Hosting              │
                      │    (Global CDN, Auto-SSL HTTPS Frontend)     │
                      │      https://<your-app>.onrender.com         │
                      └──────────────────────┬───────────────────────┘
                                             │
                       Static Web App (HTML5 / CSS3 / Vanilla JS)
                       Local User Data: window.localStorage (Isolated)
                                             │
                                             ▼
                      ┌──────────────────────────────────────────────┐
                      │         User's Laptop / Local Host           │
                      │       http://localhost:8080 (REST / WS)      │
                      │                                              │
                      │   ┌──────────────────────────────────────┐   │
                      │   │       Python Tornado HTTP & WS       │   │
                      │   │   - Universal CORS & PNA Enabled     │   │
                      │   │   - 30 Hz Telemetry Broadcast        │   │
                      │   └──────────────────┬───────────────────┘   │
                      │                      │                       │
                      │   ┌──────────────────┴───────────────────┐   │
                      │   │         ROS 2 Humble Node            │   │
                      │   │  /so101/joint_commands [JointState]  │   │
                      │   │  /so101/joint_states   [JointState]  │   │
                      │   └──────────────────┬───────────────────┘   │
                      │                      │                       │
                      │   ┌──────────────────┴───────────────────┐   │
                      │   │          Hardware Drivers            │   │
                      │   │  - SO101Driver (1 Mbps Serial UART)  │   │
                      │   │  - CameraManager (USB / RealSense)   │   │
                      │   └──────────────────┬───────────────────┘   │
                      └──────────────────────┼───────────────────────┘
                                             │
                        USB Bus (/dev/ttyACM0, /dev/video*)
                                             │
                                             ▼
                      ┌──────────────────────────────────────────────┐
                      │              Physical Hardware               │
                      │   - 6x Feetech STS3215 Servos (12V / 7.4V)   │
                      │   - USB Webcams / Intel RealSense / Orbbec   │
                      └──────────────────────────────────────────────┘
```

---

## Key Features

1. **Dual Control Modes**:
   - **Live Dynamic Sliders**: Move the physical arm in real-time as you drag or scroll sliders.
   - **Direct Numerical Input**: Exact manual degree targeting with customizable velocity (`rad/s`).
2. **Official LeRobot SO-101 Calibration**:
   - 4-step guided wizard (Torque disable → Middle pose recording → Dynamic range limits → Save to `so101_calibration.json`).
3. **Multi-Camera Vision Studio**:
   - Live streaming for standard USB / UVC webcams, Intel RealSense (RGB + Depth), Orbbec, and ESP32-CAM.
   - Real-time brightness/contrast tuning, snapshot capture, and fullscreen inspection.
4. **Autonomous Testing & Analytics**:
   - 10-target step-response dynamics test with settle detection and voltage monitoring.
   - Generates detailed, professional one-page characterization reports (`.txt`).
5. **Movement Recording & Playback**:
   - Manual lead-through recording (torque-off gravity guide) exported to clean CSV datasets.
   - High-fidelity playback engine with real-time progress tracking.
6. **Zero-Cloud Local Privacy**:
   - Every user's settings (theme mode, typography, font size, bridge address) are stored strictly in their browser's `localStorage`. No user data is sent to external cloud databases.

---

## Project Structure

```
servo_test/
├── render.yaml                          # Render Blueprint infrastructure specification
├── .gitignore                           # Git ignore rules for build, caches & rosbags
├── requirements.txt                     # Python backend & computer vision dependencies
├── README.md                            # Complete setup & deployment guide
├── PROJECT_STATUS.md                    # Project tracking and hardware specifications
├── so101_calibration.json               # Active calibrated servo offsets and limits
├── home_position.txt                    # Calibrated home pose coordinates
├── zero_position.txt                    # Zero baseline coordinates
│
├── web/                                 # Frontend (Render Static Publish Directory)
│   ├── index.html                       # Responsive Single-Page Application (SPA)
│   ├── styles.css                       # Themes (Dark/Light), layouts & CSS variables
│   ├── logo.png                         # Application branding
│   └── robot_preview.png                # Hardware visualization preview
│
├── src/                                 # ROS 2 Humble Packages
│   └── so101_servo_dynamics/
│       ├── package.xml
│       ├── setup.py
│       └── so101_servo_dynamics/
│           ├── web_control_bridge.py    # Tornado Web + WS + ROS 2 Node (Port 8080)
│           ├── so101_driver.py          # Feetech STS3215 low-level serial bus driver
│           ├── hardware_connection_node.py
│           └── servo_characterization_node.py
│
└── data/                                # Local Datasets (Ignored in Git, local only)
    ├── raw/                             # Recorded lead-through CSVs and reports
    ├── processed/                       # Filtered telemetry files
    └── rosbag/                          # ROS 2 bag storage
```

---

## Deployment to Render (Static Site)

The frontend is packaged as a pure static web application. It requires **no Node.js build step**, making deployments instantaneous and lightweight.

### Option A: Automatic Blueprint Deployment (Recommended)

1. Push your repository to GitHub (see [GitHub Setup](#github-setup) below).
2. Log in to [Render](https://render.com) and click **New +** → **Blueprint**.
3. Connect your GitHub repository.
4. Render automatically reads `render.yaml` and sets up the Static Site with:
   - **Name**: `so101-robotics-dashboard`
   - **Publish Directory**: `./web`
   - **Build Command**: *(None / Empty)*
   - **SPA Routing**: Automatic rewrite from `/*` to `/index.html`
5. Click **Apply**. Your dashboard will be live at `https://so101-robotics-dashboard.onrender.com` in seconds!

### Option B: Manual Static Site Setup

If configuring manually in the Render dashboard:
1. Click **New +** → **Static Site**.
2. Select your GitHub repository.
3. Configure settings:
   - **Name**: `so101-robotics-dashboard`
   - **Branch**: `main`
   - **Build Command**: *(Leave completely blank or enter `echo "No build required"`)*
   - **Publish Directory**: `web`
4. Under **Advanced** → **Redirects / Rewrites**:
   - **Type**: `Rewrite`
   - **Source**: `/*`
   - **Destination**: `/index.html`
5. Click **Create Static Site**.

---

## Local Robot Bridge Setup

To connect physical hardware to the dashboard (whether using the hosted Render site or local browser):

### 1. Prerequisites
- **OS**: Ubuntu Linux 22.04 LTS (x86_64 or ARM64 / Raspberry Pi)
- **Middleware**: ROS 2 Humble Desktop (`ros-humble-desktop`)
- **Python**: 3.10+
- **Serial Access**: Add your user to the `dialout` group:
  ```bash
  sudo usermod -aG dialout $USER
  ```
  *(Log out and log back in for permissions to take effect)*

### 2. Install Python Dependencies
```bash
pip3 install -r requirements.txt
```

### 3. Build ROS 2 Workspace
```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select so101_servo_dynamics
source install/setup.bash
```

### 4. Start the Local Hardware Bridge
```bash
# Sourcing ROS 2 environment
source /opt/ros/humble/setup.bash
source install/setup.bash

# Run the Tornado + ROS 2 Bridge Node
python3 src/so101_servo_dynamics/so101_servo_dynamics/web_control_bridge.py
```
Output:
```
=================================================================
   SO101 WEB CONTROLLER RUNNING AT: http://localhost:8080
=================================================================
  * Serial Hardware:   /dev/ttyACM0 @ 1000000 baud
  * ROS 2 Publisher:   /so101/joint_commands [JointState]
  * ROS 2 Publisher:   /so101/joint_states   [JointState]
  * WebSocket:         ws://localhost:8080/ws
  * Datasets Save:     data/raw/*.csv
=================================================================
```

---

## Connecting Hosted Render App to Local Robot

When accessing your dashboard through the Render HTTPS URL:
1. Open `https://<your-app>.onrender.com` in your browser.
2. In the **Connection** card or **Settings** tab, verify the **Bridge URL**:
   - Default: `http://localhost:8080`
   - If running the robot on another laptop or Raspberry Pi on your local network: enter its LAN IP (e.g., `http://192.168.1.105:8080`).
3. Click **Save Bridge URL**.

### Browser Security Note (Mixed Content & Private Network Access)
Modern browsers enforce strict security when a public HTTPS site connects to a local HTTP service (`localhost`):
- **CORS & PNA**: The bridge automatically returns `Access-Control-Allow-Origin: *` and Chrome's `Access-Control-Allow-Private-Network: true`.
- **Insecure Content Flag (If Prompted)**: If your browser blocks WebSocket (`ws://`) from an `https://` site:
  - Click the **Site Settings / Lock / Shield icon** in your browser's address bar.
  - Set **Insecure content** to **Allow**.
  - Alternatively, you can always open `http://localhost:8080` directly in your browser when running the local bridge.

---

## GitHub Setup & Automatic Deployment Workflow

Once connected, your deployment workflow is fully automated:

$$\text{Local Code Changes} \xrightarrow{\quad\text{git push}\quad} \text{GitHub Repository} \xrightarrow{\quad\text{Webhook}\quad} \text{Render Auto-Deploy} \xrightarrow{\quad\text{Instant}\quad} \text{Updated Live Site}$$

### Initialize Git and Push to GitHub:
```bash
# 1. Initialize git repository
git init

# 2. Add all tracked project files (build, install, log, and rosbags are automatically excluded by .gitignore)
git add .

# 3. Check staged files to verify no unwanted files are included
git status

# 4. Create initial commit
git commit -m "feat: initial commit of SO-101 robotics suite with Render deployment config"

# 5. Link to your GitHub repository
git remote add origin https://github.com/<your-username>/<your-repo-name>.git

# 6. Push to main branch
git branch -M main
git push -u origin main
```

---

## Verification & Testing

### 1. Test Static Frontend Locally
You can test the static build directory independently with Python's HTTP server:
```bash
python3 -m http.server 8000 --directory web
```
Open `http://localhost:8000` to verify offline static rendering.

### 2. Test ROS 2 Package Build
```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select so101_servo_dynamics
```

---

## License

MIT License. Designed and developed for open-source robotics research and automation.
