# Project Status: SO101 Servo Dynamics Characterization & MuJoCo Modelling

**Last Updated:** 2026-11-12  
**Platform:** Ubuntu 22.04 LTS | ROS 2 Humble | Python 3.10  
**Hardware:** SO101 Follower Arm (6x Feetech STS3215 Serial Bus Servos)  

---

## 1. Phase Progression Overview

| Phase | Description | Status | Validation Criteria |
| :--- | :--- | :---: | :--- |
| **Repo Research** | Inspect 4 GitHub repositories | **COMPLETED** | Verified existing interfaces, models, and ROS 2 compatibility. |
| **Phase 1** | Hardware Connection & Servo Discovery | **PASSED (100%)** | All 6 STS3215 servos (IDs 1–6) verified, online, and communicating via `/dev/ttyACM0` at 1 Mbps. |
| **Phase 2** | ROS 2 Communication & Web Control UI | **COMPLETED** | Web UI on `http://localhost:8080`, `/so101/joint_commands` (Pub) and `/so101/joint_states` (Sub) active. |
| **Phase 3** | Individual Servo Characterization | **READY** | Step response, velocity limits, rise/settling time, repeatability testing tools built. |
| **Phase 4** | Real-Time Data Collection | **READY** | Web UI CSV recorder + high-frequency telemetry logging enabled. |
| **Phase 5** | Mathematical System Identification | **PENDING** | First/second-order model fitting, delay estimation, parameter estimation report. |
| **Phase 6** | MuJoCo Servo Dynamics Simulation | **PENDING** | MuJoCo MJCF model with identified actuator dynamics, damping, and limits. |
| **Phase 7** | Real Hardware vs MuJoCo Validation | **PENDING** | Real vs Sim trajectory tracking, RMSE < threshold, comparative validation report. |

---

## 2. Hardware Verification Summary

- **Serial Port:** `/dev/ttyACM0` (QinHeng USB Single Serial / Waveshare Bus Servo Driver `1a86:55d3`)
- **Baudrate:** `1,000,000` baud (1 Mbps)
- **Active Physical Servos (All 6 ONLINE):**
  - **ID 1 (`shoulder_pan`):** Model 777 (STS3215) - **ONLINE**
  - **ID 2 (`shoulder_lift`):** Model 777 (STS3215) - **ONLINE**
  - **ID 3 (`elbow_flex`):** Model 777 (STS3215) - **ONLINE**
  - **ID 4 (`wrist_flex`):** Model 777 (STS3215) - **ONLINE**
  - **ID 5 (`wrist_roll`):** Model 777 (STS3215) - **ONLINE**
  - **ID 6 (`gripper`):** Model 777 (STS3215) - **ONLINE**
- **Telemetry Frequency:** 30 Hz continuous feedback on `/so101/joint_states`

---

## 3. Web UI & ROS 2 Architecture

- **Web UI Location:** `http://localhost:8080` (Hosted by `web_control_bridge`)
- **ROS 2 Launch Command:**
  ```bash
  source /opt/ros/humble/setup.bash
  source /home/moulie/Downloads/servo_test/so101_servo_dynamics/ros2_ws/install/setup.bash
  ros2 launch so101_servo_dynamics web_control.launch.py
  ```
- **Features Implemented:**
  1. Dropdown servo selection (Joints 1 to 6).
  2. Live position slider & numeric angle control with joint limits.
  3. Real-time physical robot motion via `/so101/joint_commands`.
  4. Real-time telemetry plot (Target vs Actual position, tracking error, velocity).
  5. Built-in CSV recording (saves to `data/raw/*.csv` with 1-click download).
