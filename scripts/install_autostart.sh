#!/usr/bin/env bash
# ==============================================================================
# Installer for SO-101 Background Auto-Start Service
# Configures systemd so the bridge starts automatically on laptop boot
# ==============================================================================

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SERVICE_DIR="$HOME/.config/systemd/user"

mkdir -p "$SERVICE_DIR"
cp "$WORKSPACE_DIR/scripts/so101-bridge.service" "$SERVICE_DIR/so101-bridge.service"

systemctl --user daemon-reload
systemctl --user enable so101-bridge.service
systemctl --user start so101-bridge.service

echo ""
echo "================================================================="
echo " ✔ SO-101 Bridge Auto-Start Service installed successfully!"
echo " The bridge is now running silently in the background."
echo " Status check: systemctl --user status so101-bridge.service"
echo " Stop service: systemctl --user stop so101-bridge.service"
echo "================================================================="
