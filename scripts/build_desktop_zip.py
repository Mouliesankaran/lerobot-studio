#!/usr/bin/env python3
"""Rebuild web/so101_studio_desktop.zip from the single sources of truth.
Run from anywhere:  python3 scripts/build_desktop_zip.py
The package is FLAT and contains the web UI, so the bridge can serve it on any OS."""
import shutil, zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
WEB = ROOT / "web"
CAM = ROOT / "src/so101_servo_dynamics/so101_servo_dynamics/camera_backend.py"

# 1. keep web/ copies of the launchers in sync (web/ is what Render publishes)
for name in ("bridge_standalone.py", "run_linux.sh", "run_windows.bat", "requirements-bridge.txt"):
    shutil.copy2(ROOT / name, WEB / name)
shutil.copy2(CAM, WEB / "camera_backend.py")

files = {
    "bridge_standalone.py": ROOT / "bridge_standalone.py",
    "camera_backend.py": CAM,
    "run_linux.sh": ROOT / "run_linux.sh",
    "run_windows.bat": ROOT / "run_windows.bat",
    "requirements-bridge.txt": ROOT / "requirements-bridge.txt",
    "README.txt": WEB / "README_WINDOWS.txt",
    "home_position.txt": ROOT / "home_position.txt",
    "zero_position.txt": ROOT / "zero_position.txt",
    "so101_calibration.json": ROOT / "so101_calibration.json",
    "index.html": WEB / "index.html",
    "styles.css": WEB / "styles.css",
    "logo.png": WEB / "logo.png",
    "robot_preview.png": WEB / "robot_preview.png",
}
out = WEB / "so101_studio_desktop.zip"
with zipfile.ZipFile(out, "w", zipfile.ZIP_DEFLATED) as z:
    for arc, src in files.items():
        info = zipfile.ZipInfo(arc, date_time=(2026, 11, 14, 0, 0, 0))
        info.compress_type = zipfile.ZIP_DEFLATED
        info.external_attr = (0o755 if arc.endswith(".sh") else 0o644) << 16   # keep +x on Linux
        z.writestr(info, src.read_bytes())
print(f"built {out}  ({out.stat().st_size/1024:.0f} KB, {len(files)} files)")
