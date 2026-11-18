#!/usr/bin/env bash
# ==============================================================================
# Setup Systemd Autostart Service for SO-101 (Starts automatically on PC Boot)
# ==============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SERVICE_DIR="$HOME/.config/systemd/user"
SERVICE_FILE="$SERVICE_DIR/so101_bridge.service"

mkdir -p "$SERVICE_DIR"

cat <<EOF > "$SERVICE_FILE"
[Unit]
Description=SO-101 Robotics Hardware Bridge Server (Localhost 8080)
After=network.target

[Service]
Type=simple
WorkingDirectory=$SCRIPT_DIR
ExecStart=$SCRIPT_DIR/run_ubuntu.sh --no-browser
Restart=always
RestartSec=3
KillMode=process

[Install]
WantedBy=default.target
EOF

echo "[*] Created service file at: $SERVICE_FILE"

# Reload systemd user daemon
systemctl --user daemon-reload

# Enable service to run on boot/login
systemctl --user enable so101_bridge.service

# Start the service right now
systemctl --user restart so101_bridge.service

echo "================================================================="
echo " [OK] Systemd Autostart Installed & Enabled Successfully!"
echo " Status: Running in background on http://localhost:8080"
echo " PC on panna pothum, automatic-a start aagidum (24/7 Always Online)."
echo ""
echo " Useful Commands:"
echo "   systemctl --user status so101_bridge   (Check status)"
echo "   systemctl --user stop so101_bridge     (Stop server)"
echo "   systemctl --user start so101_bridge    (Start server)"
echo "   systemctl --user restart so101_bridge  (Restart server)"
echo "   systemctl --user disable so101_bridge  (Disable autostart)"
echo "================================================================="
