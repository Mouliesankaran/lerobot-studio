#!/bin/bash
# ========================================================
# SO-101 Local Backup Restore Script
# Restores working code directly from local zip backup (no git needed)
# ========================================================

TARGET_ZIP="${1:-local_backups/so101_working_backup_latest.zip}"

if [ ! -f "$TARGET_ZIP" ]; then
    echo "[ERROR] Backup zip file not found at: $TARGET_ZIP"
    exit 1
fi

echo "========================================================"
echo "Restoring SO-101 system from: $TARGET_ZIP"
echo "========================================================"

unzip -o "$TARGET_ZIP" -x "local_backups/*" ".git/*"

echo ""
echo "✔ Restore complete! All working application files are restored."
