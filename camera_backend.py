#!/usr/bin/env python3
"""
SO-101 shared camera backend (Windows / Linux / macOS).

Single source of truth used by BOTH:
  * bridge_standalone.py       (pure-Python bridge, no ROS 2)
  * web_control_bridge.py      (ROS 2 Humble bridge)

Supported sources
  usb        - any UVC webcam via OpenCV (DirectShow/MSMF on Windows, V4L2 on Linux,
               AVFoundation on macOS).  RGB is real.  "Depth" is a *pseudo-depth*
               (luminance colour-map) and is always flagged as such.
  realsense  - Intel RealSense D400 / L500 via pyrealsense2 (RGB + metric 16-bit depth,
               depth aligned to colour).
  orbbec     - Orbbec Femto / Gemini / Astra via pyorbbecsdk (RGB + metric depth).
               EXPERIMENTAL: written against the official SDK samples, not hardware-tested.
  network    - ESP32-CAM / IP camera (HTTP MJPEG / RTSP) via OpenCV.  RGB only.
  synthetic  - generated RGB + real metric depth, for CI / acceptance tests
               (enabled with SO101_TEST_CAMERA=1 or --test-camera).

Nothing here hard-codes an OS path: device discovery uses sysfs on Linux, index probing
elsewhere, and every OpenCV backend is tried in an OS-appropriate order.
"""

import os
import sys
import glob
import re
import shutil
import subprocess
import time
import threading
from typing import Any, Dict, List, Optional, Tuple

# Windows MSMF takes ~30 s to open a camera unless HW transforms are disabled.
# Must be set BEFORE cv2 is imported.
os.environ.setdefault("OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS", "0")

try:
    import cv2
    import numpy as np
    CV2_AVAILABLE = True
    CV2_ERROR = ""
except Exception as _e:  # ImportError or broken binary wheel
    cv2 = None  # type: ignore
    np = None   # type: ignore
    CV2_AVAILABLE = False
    CV2_ERROR = str(_e)

import site
for _sp in (site.getusersitepackages(), os.path.expanduser("~/.local/lib/python3.10/site-packages")):
    if _sp and os.path.isdir(_sp) and _sp not in sys.path:
        sys.path.insert(0, _sp)

try:
    import pyrealsense2 as rs
    REALSENSE_AVAILABLE = True
except Exception:
    rs = None  # type: ignore
    REALSENSE_AVAILABLE = False

try:
    import pyorbbecsdk as ob  # type: ignore
    ORBBEC_AVAILABLE = True
except Exception:
    ob = None  # type: ignore
    ORBBEC_AVAILABLE = False

IS_WINDOWS = sys.platform.startswith("win")
IS_LINUX = sys.platform.startswith("linux")
IS_MAC = sys.platform == "darwin"

# Depth colouring range (metres) for metric depth cameras.
DEPTH_MIN_M = 0.15
DEPTH_MAX_M = 4.0

STREAM_MODES = ("rgb", "depth", "side_by_side")


def _cv_backends() -> List[int]:
    """OpenCV capture backends to try, best first, for this OS."""
    if not CV2_AVAILABLE:
        return []
    if IS_WINDOWS:
        return [cv2.CAP_DSHOW, cv2.CAP_MSMF, cv2.CAP_ANY]
    if IS_LINUX:
        return [cv2.CAP_V4L2, cv2.CAP_ANY]
    if IS_MAC:
        return [cv2.CAP_AVFOUNDATION, cv2.CAP_ANY]
    return [cv2.CAP_ANY]


def colorize_depth(depth_u16, depth_scale: float = 0.001,
                   min_m: float = DEPTH_MIN_M, max_m: float = DEPTH_MAX_M):
    """uint16 depth -> BGR colour image.  Uses the camera's real depth scale
    (RealSense D405 = 0.0001 m/unit, D435 = 0.001 m/unit) so colours are metric.
    Invalid pixels (0, out-of-range, saturated noise) are black."""
    meters = depth_u16.astype(np.float32) * float(depth_scale)
    norm = np.clip((meters - min_m) / max(1e-6, (max_m - min_m)), 0.0, 1.0)
    img8 = (255 - (norm * 255)).astype(np.uint8)  # near = warm
    out = cv2.applyColorMap(img8, cv2.COLORMAP_JET)
    valid = (depth_u16 > 100) & (depth_u16 < 8000) & (depth_u16 != 65535)
    out[~valid] = 0
    return out


def pseudo_depth(bgr):
    """Luminance colour-map. NOT metric depth - always watermarked by the caller."""
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    return cv2.applyColorMap(gray, cv2.COLORMAP_TURBO)


def _as_bgr8(frame):
    """Guarantee HxWx3 uint8 (some V4L2 nodes hand back gray / 16-bit / BGRA)."""
    if frame is None:
        return frame
    if frame.dtype != np.uint8:
        frame = cv2.convertScaleAbs(frame, alpha=255.0 / max(1.0, float(frame.max())))
    if frame.ndim == 2:
        return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
    if frame.shape[2] == 4:
        return cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
    return frame


class FFmpegV4L2Capture:
    """High-speed, zero-latency direct V4L2 streaming capture using FFmpeg (the ffplay method).
    Bypasses unstable proprietary SDK drivers, streams synchronized RGB + 16-bit Metric Depth,
    and cleanly trims bottom sensor metadata scanlines."""
    def __init__(self, color_node: Optional[str], depth_node: Optional[str] = None,
                 color_size: Tuple[int, int] = (640, 480), depth_size: Tuple[int, int] = (640, 480),
                 is_realsense: bool = False, depth_scale_m: float = 0.001):
        self.color_node = color_node
        self.depth_node = depth_node
        self.cw, self.ch = color_size
        self.dw, self.dh = depth_size
        self.is_realsense = is_realsense
        self.depth_scale_m = depth_scale_m
        self.running = False
        self.proc_c: Optional[subprocess.Popen] = None
        self.proc_d: Optional[subprocess.Popen] = None
        self.thread_c: Optional[threading.Thread] = None
        self.thread_d: Optional[threading.Thread] = None
        self.latest_rgb: Optional[np.ndarray] = None
        self.latest_depth_raw: Optional[np.ndarray] = None
        self.lock = threading.Lock()
        self.frame_seq = 0
        self.dropped_frames = 0

    def start(self):
        self.running = True
        if self.color_node and os.path.exists(self.color_node):
            cmd_c = [
                "ffmpeg", "-hide_banner", "-loglevel", "error",
                "-fflags", "nobuffer", "-flags", "low_delay",
                "-f", "v4l2", "-video_size", f"{self.cw}x{self.ch}",
                "-i", self.color_node,
                "-f", "rawvideo", "-pix_fmt", "bgr24", "pipe:1"
            ]
            self.proc_c = subprocess.Popen(cmd_c, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=self.cw * self.ch * 3 * 2)
            self.thread_c = threading.Thread(target=self._worker_color, daemon=True, name="ffmpeg-v4l2-c")
            self.thread_c.start()

        if self.depth_node and os.path.exists(self.depth_node):
            cmd_d = [
                "ffmpeg", "-hide_banner", "-loglevel", "error",
                "-fflags", "nobuffer", "-flags", "low_delay",
                "-f", "v4l2", "-input_format", "gray16le", "-video_size", f"{self.dw}x{self.dh}",
                "-i", self.depth_node,
                "-f", "rawvideo", "-pix_fmt", "gray16le", "pipe:1"
            ]
            self.proc_d = subprocess.Popen(cmd_d, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=self.dw * self.dh * 2 * 2)
            self.thread_d = threading.Thread(target=self._worker_depth, daemon=True, name="ffmpeg-v4l2-d")
            self.thread_d.start()

    def _worker_color(self):
        fsize = self.cw * self.ch * 3
        while self.running and self.proc_c is not None:
            raw = self.proc_c.stdout.read(fsize)
            if len(raw) != fsize:
                if not self.running:
                    break
                time.sleep(0.01)
                continue
            img = np.frombuffer(raw, dtype=np.uint8).reshape((self.ch, self.cw, 3)).copy()
            if self.is_realsense and self.ch > 100:
                # Remove RealSense bottom sensor register scanlines (horizontal line bug fix)
                img = cv2.resize(img[:-8, :], (self.cw, self.ch), interpolation=cv2.INTER_LINEAR)
            with self.lock:
                self.latest_rgb = img
                self.frame_seq += 1

    def _worker_depth(self):
        fsize = self.dw * self.dh * 2
        while self.running and self.proc_d is not None:
            raw = self.proc_d.stdout.read(fsize)
            if len(raw) != fsize:
                if not self.running:
                    break
                time.sleep(0.01)
                continue
            arr = np.frombuffer(raw, dtype=np.uint16).reshape((self.dh, self.dw)).copy()
            with self.lock:
                self.latest_depth_raw = arr
                if self.color_node is None or self.latest_rgb is None:
                    meters = arr.astype(np.float32) * float(self.depth_scale_m)
                    norm = np.clip((meters - 0.15) / 3.85, 0.0, 1.0)
                    img8 = (255 - (norm * 255)).astype(np.uint8)
                    vis = cv2.applyColorMap(img8, cv2.COLORMAP_JET)
                    vis[arr == 0] = 0
                    self.latest_rgb = vis
                    self.frame_seq += 1

    def stop(self):
        self.running = False
        for p in (self.proc_c, self.proc_d):
            if p is not None:
                try:
                    p.terminate()
                    p.wait(timeout=0.5)
                except Exception:
                    try:
                        p.kill()
                    except Exception:
                        pass
        self.proc_c = None
        self.proc_d = None
        if self.thread_c and self.thread_c.is_alive():
            self.thread_c.join(timeout=0.3)
        if self.thread_d and self.thread_d.is_alive():
            self.thread_d.join(timeout=0.3)
        self.thread_c = None
        self.thread_d = None


class CameraManager:
    """Thread-safe multi-camera manager with explicit errors and real depth."""

    def __init__(self):
        self.is_connected = False
        self.camera_type = "usb"
        self.device_id = "0"
        self.width = 640
        self.height = 480
        self.target_fps = 30
        self.stream_mode = "rgb"

        self.brightness = 50
        self.contrast = 50
        self.exposure = 50

        self.cap = None
        self.depth_pipe_proc = None
        self.depth_thread = None
        self._depth_stop_evt = threading.Event()
        self.rs_pipeline = None
        self.rs_align = None
        self.rs_depth_scale = 0.001
        self.ob_pipeline = None
        self.ob_depth_scale = 1.0
        self.ffmpeg_capture: Optional[FFmpegV4L2Capture] = None
        self._depth_scale_m = 0.001          # metres per raw depth unit (set per camera)
        self._v4l2_role_cache: Dict[str, str] = {}
        self._synthetic = False

        self.is_running = False
        self.thread: Optional[threading.Thread] = None
        self.lock = threading.Lock()
        self._connect_lock = threading.Lock()

        self.latest_rgb = None
        self.latest_depth_color = None      # BGR visualisation
        self.latest_depth_raw = None        # uint16 (metric cameras only)
        self.has_depth_hardware = False
        self.frame_seq = 0

        self.actual_fps = 0.0
        self.dropped_frames = 0
        self.last_error = ""
        self._fps_count = 0
        self._fps_t0 = time.time()

        self._jpeg_cache: Tuple[Any, Optional[bytes]] = (None, None)
        self._enum_cache: Tuple[float, List[Dict[str, Any]]] = (0.0, [])

    # ------------------------------------------------------------------ discovery
    def diagnostics(self) -> Dict[str, Any]:
        info: Dict[str, Any] = {
            "platform": sys.platform,
            "python": sys.version.split()[0],
            "opencv": (cv2.__version__ if CV2_AVAILABLE else None),
            "opencv_error": CV2_ERROR or None,
            "pyrealsense2": REALSENSE_AVAILABLE,
            "pyorbbecsdk": ORBBEC_AVAILABLE,
            "video_nodes": [],
            "video_access_ok": None,
        }
        if IS_LINUX:
            nodes = sorted(glob.glob("/dev/video*"))
            info["video_nodes"] = nodes
            info["video_access_ok"] = all(os.access(n, os.R_OK | os.W_OK) for n in nodes) if nodes else None
            if nodes and not info["video_access_ok"]:
                info["hint"] = "Add your user to the 'video' group: sudo usermod -aG video $USER (then log out/in)."
        return info

    @staticmethod
    def _linux_usb_vendor(idx: int) -> str:
        """USB idVendor of /dev/videoN (8086 = Intel RealSense, 2bc5 = Orbbec). '' if not USB."""
        try:
            p = os.path.realpath(f"/sys/class/video4linux/video{idx}/device")
            for _ in range(5):
                f = os.path.join(p, "idVendor")
                if os.path.isfile(f):
                    with open(f) as fh:
                        return fh.read().strip().lower()
                p = os.path.dirname(p)
        except Exception:
            pass
        return ""

    def _v4l2_node_role(self, path: str, name: str) -> str:
        """'color' | 'depth' for a RealSense/Orbbec node.  Name first, then the formats the node
        really offers (Z16/GREY/Y16 = depth/IR, YUYV/MJPG/NV12 = colour)."""
        low = name.lower()
        if re.search(r"\b(rgb|color|colour)\b", low):
            return "color"
        if re.search(r"\b(depth|infrared|ir)\b", low):
            return "depth"
        if path in self._v4l2_role_cache:
            return self._v4l2_role_cache[path]
        role = ""
        if shutil.which("v4l2-ctl"):
            try:
                out = subprocess.run(["v4l2-ctl", "-d", path, "--list-formats"], capture_output=True,
                                     text=True, timeout=1.5).stdout.upper()
                if any(f"'{k}" in out for k in ("YUYV", "MJPG", "NV12", "UYVY", "YUY2")):
                    role = "color"
                elif any(f"'{k}" in out for k in ("Z16", "GREY", "Y16", "Y8I", "Y12I", "Y10")):
                    role = "depth"
            except Exception:
                pass
        role = role or "color"
        self._v4l2_role_cache[path] = role
        return role

    def _linux_video_devices(self) -> List[Dict[str, Any]]:
        """Dynamic Linux V4L2 device scanner using kernel ioctl VIDIOC_ENUM_FMT.
        Accurately discovers RealSense (Color & Depth), Orbbec (Color & Depth),
        and UVC Webcams without cluttering metadata nodes or conflicting ports."""
        found_devs = []
        import fcntl, struct
        VIDIOC_ENUM_FMT = 0xc0405602
        V4L2_BUF_TYPE_VIDEO_CAPTURE = 1

        for dev in sorted(glob.glob("/dev/video*"), key=lambda x: int(x.replace("/dev/video", ""))):
            name_path = f"/sys/class/video4linux/{os.path.basename(dev)}/name"
            name = open(name_path).read().strip() if os.path.exists(name_path) else ""
            try:
                fd = os.open(dev, os.O_RDWR | os.O_NONBLOCK)
            except Exception:
                continue
            fmts = []
            idx = 0
            while True:
                buf = bytearray(struct.pack("III32sIIII", idx, V4L2_BUF_TYPE_VIDEO_CAPTURE, 0, b"", 0, 0, 0, 0))
                try:
                    fcntl.ioctl(fd, VIDIOC_ENUM_FMT, buf)
                    _, _, _, desc, pixfmt, _, _, _ = struct.unpack("III32sIIII", buf)
                    fourcc = "".join(chr((pixfmt >> (8 * i)) & 0xFF) for i in range(4))
                    desc_str = desc.split(b"\x00")[0].decode("ascii", errors="ignore")
                    fmts.append((fourcc, desc_str))
                    idx += 1
                except Exception:
                    break
            os.close(fd)
            if not fmts:
                continue
            low_name = name.lower()
            family = "usb"
            if "orbbec" in low_name:
                family = "orbbec"
            elif "realsense" in low_name:
                family = "realsense"

            is_depth = any(f[0] in ("Z16 ", "Y16 ") for f in fmts)
            is_color = any(f[0] in ("YUYV", "MJPG", "RGB3", "BGR3") for f in fmts)
            is_ir = any(f[0] in ("GREY", "UYVY") for f in fmts) and not is_color and not is_depth

            found_devs.append({
                "id": dev, "name_raw": name, "family": family,
                "is_depth": is_depth, "is_color": is_color, "is_ir": is_ir,
                "fmts": [f[0] for f in fmts]
            })

        final_devices = []
        # RealSense devices
        rs_color = next((d for d in found_devs if d["family"] == "realsense" and d["is_color"]), None)
        rs_depth = next((d for d in found_devs if d["family"] == "realsense" and d["is_depth"]), None)
        if rs_color:
            nid = rs_color["id"]
            final_devices.append({
                "id": nid, "name": f"Camera Port 1: RGB Color Camera ({nid})",
                "type": "realsense", "role": "color", "has_rgb": True, "has_depth": True,
                "default_res": "640x480", "accessible": os.access(nid, os.R_OK | os.W_OK)
            })
        if rs_depth:
            nid = rs_depth["id"]
            final_devices.append({
                "id": nid, "name": f"Camera Port 2: 3D Depth Sensor ({nid})",
                "type": "realsense", "role": "depth", "has_rgb": False, "has_depth": True,
                "default_res": "640x480", "accessible": os.access(nid, os.R_OK | os.W_OK)
            })

        # Orbbec devices
        ob_color = next((d for d in found_devs if d["family"] == "orbbec" and d["is_color"]), None)
        ob_depth = next((d for d in found_devs if d["family"] == "orbbec" and d["is_depth"]), None)
        if ob_color:
            nid = ob_color["id"]
            final_devices.append({
                "id": nid, "name": f"Camera Port 1: RGB Color Camera ({nid})",
                "type": "orbbec", "role": "color", "has_rgb": True, "has_depth": True,
                "default_res": "1280x720", "accessible": os.access(nid, os.R_OK | os.W_OK)
            })
        if ob_depth:
            nid = ob_depth["id"]
            final_devices.append({
                "id": nid, "name": f"Camera Port 2: 3D Depth Sensor ({nid})",
                "type": "orbbec", "role": "depth", "has_rgb": False, "has_depth": True,
                "default_res": "1024x512", "accessible": os.access(nid, os.R_OK | os.W_OK)
            })

        # USB webcams
        for d in found_devs:
            if d["family"] == "usb" and d["is_color"]:
                nid = d["id"]
                raw = d["name_raw"]
                final_devices.append({
                    "id": nid, "name": f"{raw} ({nid})",
                    "type": "usb", "role": "color", "has_rgb": True, "has_depth": False,
                    "default_res": "640x480", "accessible": os.access(nid, os.R_OK | os.W_OK)
                })
        return final_devices

    def _windows_device_names(self) -> List[str]:
        """Query real Windows device names for cameras using PowerShell CIM (Win 10/11 native)."""
        names: List[str] = []
        if not IS_WINDOWS:
            return names
        try:
            cmd = ["powershell", "-NoProfile", "-NonInteractive", "-Command",
                   "Get-CimInstance Win32_PnPEntity | Where-Object { $_.PNPClass -in @('Camera','Image') } | Select-Object -ExpandProperty Name"]
            res = subprocess.run(cmd, capture_output=True, text=True, timeout=2.5)
            if res.returncode == 0:
                for line in res.stdout.splitlines():
                    l = line.strip()
                    if l and l not in names:
                        names.append(l)
        except Exception:
            pass
        return names

    def _probe_indices(self, max_idx: int = 8) -> List[Dict[str, Any]]:
        win_names = self._windows_device_names()
        out = []
        for idx in range(max_idx):
            if self.is_connected and self.camera_type in ("usb", "orbbec", "realsense") and str(idx) == str(self.device_id):
                out.append(self._usb_entry(idx, win_names))
                continue
            ok = False
            for be in _cv_backends()[:2]:
                try:
                    c = cv2.VideoCapture(idx, be)
                    ok = c.isOpened()
                    c.release()
                except Exception:
                    ok = False
                if ok:
                    break
            if ok:
                out.append(self._usb_entry(idx, win_names))
        return out

    @staticmethod
    def _usb_entry(idx: int, win_names: Optional[List[str]] = None) -> Dict[str, Any]:
        dev_name = f"Camera {idx}"
        dev_type = "usb"
        has_depth = False
        if win_names and idx < len(win_names):
            raw_name = win_names[idx]
            low = raw_name.lower()
            if "orbbec" in low or "femto" in low or "astra" in low:
                dev_type = "orbbec"
                has_depth = True
                dev_name = f"Orbbec 3D: {raw_name} (Camera {idx})"
            elif "realsense" in low or "intel" in low or "d455" in low or "d435" in low:
                dev_type = "realsense"
                has_depth = True
                dev_name = f"Intel RealSense: {raw_name} (Camera {idx})"
            else:
                dev_name = f"{raw_name} (Camera {idx})"
        return {"id": str(idx), "name": dev_name, "type": dev_type, "has_rgb": True,
                "has_depth": has_depth, "default_res": "640x480"}

    def list_devices(self, force: bool = False) -> List[Dict[str, Any]]:
        now = time.time()
        if not force and now - self._enum_cache[0] < 5.0 and self._enum_cache[1]:
            return list(self._enum_cache[1])

        devices: List[Dict[str, Any]] = []

        if IS_LINUX:
            try:
                devices = self._linux_video_devices()
            except Exception as e:
                self.last_error = f"Linux video enumeration failed: {e}"
        else:
            if REALSENSE_AVAILABLE:
                try:
                    for dev in rs.context().query_devices():
                        name = dev.get_info(rs.camera_info.name)
                        sn = dev.get_info(rs.camera_info.serial_number)
                        devices.append({"id": f"realsense:{sn}", "name": f"Intel RealSense {name} (S/N: {sn})",
                                        "type": "realsense", "has_rgb": True, "has_depth": True,
                                        "default_res": "640x480"})
                except Exception as e:
                    self.last_error = f"RealSense enumeration failed: {e}"

            if ORBBEC_AVAILABLE:
                try:
                    ctx = ob.Context()
                    dl = ctx.query_devices()
                    for i in range(dl.get_count()):
                        devices.append({"id": f"orbbec:{i}", "name": f"Orbbec {dl.get_device_name_by_index(i)}"
                                        f" (#{i})", "type": "orbbec", "has_rgb": True, "has_depth": True,
                                        "default_res": "640x480"})
                except Exception as e:
                    self.last_error = f"Orbbec enumeration failed: {e}"

            if CV2_AVAILABLE:
                try:
                    devices += self._probe_indices()
                except Exception as e:
                    self.last_error = f"USB enumeration failed: {e}"

        if os.environ.get("SO101_TEST_CAMERA") == "1":
            devices.append({"id": "synthetic", "name": "Synthetic Test Camera (RGB + metric depth)",
                            "type": "synthetic", "has_rgb": True, "has_depth": True,
                            "default_res": "640x480"})

        # Only add network stream if no physical hardware cameras were detected
        if not devices:
            devices.append({"id": "network_esp32", "name": "ESP32-CAM / IP Stream", "type": "network",
                            "has_rgb": True, "has_depth": False, "default_res": "640x480"})
        self._enum_cache = (now, devices)
        return list(devices)

    # ------------------------------------------------------------------ connect
    def connect(self, device_id: str, camera_type: str = "usb", width: int = 640, height: int = 480,
                fps: int = 30, custom_url: Optional[str] = None) -> Dict[str, Any]:
        with self._connect_lock:
            self.disconnect()
            time.sleep(0.3)
            self.device_id, self.camera_type = str(device_id), camera_type
            self.width, self.height, self.target_fps = int(width), int(height), int(fps)
            self.has_depth_hardware = False
            self.last_error = ""
            self.dropped_frames = 0

            if camera_type != "synthetic" and not CV2_AVAILABLE:
                return self._fail(f"OpenCV is not usable ({CV2_ERROR or 'not installed'}). "
                                  "Run: pip install opencv-python numpy")
            try:
                # 1. Native SDK pipelines (RealSense pyrealsense2 & Orbbec pyorbbecsdk) give best synchronized RGB + metric depth
                if camera_type == "realsense" and REALSENSE_AVAILABLE:
                    self._open_realsense(device_id, width, height, fps)
                elif camera_type == "orbbec" and ORBBEC_AVAILABLE:
                    self._open_orbbec(device_id, width, height, fps)
                # 2. Linux FFmpeg V4L2 streaming engine (matches ffplay speed, latency, and quality)
                elif IS_LINUX and shutil.which("ffmpeg") and (camera_type in ("realsense", "orbbec", "usb") or str(device_id).startswith("/dev/video")):
                    self._open_ffmpeg_v4l2(str(device_id), camera_type, width, height, fps)
                elif camera_type == "realsense":
                    if REALSENSE_AVAILABLE:
                        self._open_realsense(device_id, width, height, fps)
                    else:
                        self._open_usb(device_id, width, height, fps)
                elif camera_type == "orbbec":
                    if ORBBEC_AVAILABLE:
                        self._open_orbbec(device_id, width, height, fps)
                    else:
                        self._open_usb(device_id, width, height, fps)
                elif camera_type == "synthetic":
                    self._synthetic = True
                    self.has_depth_hardware = True
                elif camera_type == "network":
                    self._open_network(custom_url)
                else:
                    self._open_usb(device_id, width, height, fps)
            except Exception as e:
                self.disconnect()
                return self._fail(str(e))

            self.is_connected = True
            self.is_running = True
            self.frame_seq = 0
            self.thread = threading.Thread(target=self._capture_worker, daemon=True, name="so101-cam")
            self.thread.start()
            return {"success": True,
                    "message": f"Connected to {self.camera_type} on {self.device_id} ({self.width}x{self.height} @ {self.target_fps} FPS)",
                    "camera_type": self.camera_type, "has_depth": self.has_depth_hardware,
                    "depth_is_real": self.has_depth_hardware}

    def _open_ffmpeg_v4l2(self, device_id: str, camera_type: str, width: int, height: int, fps: int):
        devices = self._linux_video_devices()
        dev_info = next((d for d in devices if d["id"] == device_id), None)
        fam = camera_type
        if dev_info:
            fam = dev_info.get("type", camera_type)

        color_node = None
        depth_node = None
        cw, ch = width, height
        dw, dh = width, height
        is_rs = (fam == "realsense")
        is_ob = (fam == "orbbec")

        if is_rs:
            col = next((d["id"] for d in devices if d["type"] == "realsense" and d["role"] == "color"), None)
            color_node = col or device_id
            depth_node = None  # RealSense USB hardware rejects dual raw V4L2 opens; native SDK handles dual depth+RGB
            cw, ch = (640, 480) if width not in (1280, 848) else (width, height)
            dw, dh = (640, 480)
            self._depth_scale_m = 0.001
            self.has_depth_hardware = False
        elif is_ob:
            col = next((d["id"] for d in devices if d["type"] == "orbbec" and d["role"] == "color"), None)
            dep = next((d["id"] for d in devices if d["type"] == "orbbec" and d["role"] == "depth"), None)
            color_node = col or device_id
            depth_node = dep
            cw, ch = (1280, 720)
            dw, dh = (1024, 512)
            self._depth_scale_m = 0.001
            self.has_depth_hardware = (depth_node is not None)
        else:
            color_node = device_id if device_id.startswith("/dev/video") else "/dev/video0"
            depth_node = None
            cw, ch = width, height
            dw, dh = width, height
            self.has_depth_hardware = False

        if dev_info and dev_info.get("role") == "depth" and not color_node:
            depth_node = device_id
            self.has_depth_hardware = True

        self.width, self.height = cw, ch
        self.ffmpeg_capture = FFmpegV4L2Capture(
            color_node=color_node, depth_node=depth_node,
            color_size=(cw, ch), depth_size=(dw, dh),
            is_realsense=is_rs, depth_scale_m=self._depth_scale_m
        )
        self.ffmpeg_capture.start()

        # Wait up to 1.5s for initial frame
        t0 = time.time()
        while time.time() - t0 < 1.5:
            with self.ffmpeg_capture.lock:
                if self.ffmpeg_capture.latest_rgb is not None:
                    self.latest_rgb = self.ffmpeg_capture.latest_rgb
                    self.latest_depth_raw = self.ffmpeg_capture.latest_depth_raw
                    break
            time.sleep(0.05)

    def _fail(self, msg: str) -> Dict[str, Any]:
        self.last_error = msg
        return {"success": False, "message": msg}

    def _resolve_color_node(self, alias: str):
        """Map an alias ('realsense', 'orbbec', 'usb', 'default') to a working COLOUR node."""
        want = next((t for t in ("realsense", "orbbec") if t in alias.lower()), None)
        nodes = [d for d in self.list_devices(force=True)
                 if str(d.get("id", "")).startswith("/dev/video") or str(d.get("id", "")).isdigit()]
        pick = lambda f: next((d["id"] for d in nodes if d.get("accessible", True) and f(d)), None)
        cand = (pick(lambda d: want and d["type"] == want and not d.get("has_depth"))
                or pick(lambda d: not d.get("has_depth")) or (nodes[0]["id"] if nodes else 0))
        return int(cand) if str(cand).isdigit() else cand

    def _ensure_color_node(self, node: str) -> str:
        """If the user picked a Depth/IR node, OpenCV would time out (BGR requested from Z16).
        Re-point to the colour node of the same vendor."""
        if not IS_LINUX:
            return node
        info = next((d for d in self.list_devices() if d.get("id") == node), None)
        if info and info.get("has_depth"):
            alt = next((d["id"] for d in self.list_devices() if d["type"] == info["type"]
                        and not d.get("has_depth") and d["id"].startswith("/dev/video")), None)
            if alt:
                self.last_error = f"{node} is a depth/IR node; using colour node {alt} instead."
                return alt
        return node

    def _start_depth_pipe(self, depth_node: str = "/dev/video2", width: int = 640, height: int = 480):
        self._stop_depth_pipe()
        self._depth_stop_evt.clear()
        cmd = [
            "ffmpeg", "-f", "v4l2", "-input_format", "gray16le", "-video_size", f"{width}x{height}",
            "-i", depth_node, "-f", "rawvideo", "-pix_fmt", "gray16le", "pipe:1"
        ]
        try:
            self.depth_pipe_proc = subprocess.Popen(
                cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=width * height * 2 * 2
            )
            fsize = width * height * 2
            def depth_reader():
                while not self._depth_stop_evt.is_set() and self.depth_pipe_proc is not None:
                    try:
                        raw = self.depth_pipe_proc.stdout.read(fsize)
                        if len(raw) == fsize:
                            arr = np.frombuffer(raw, dtype=np.uint16).reshape((height, width))
                            self._cached_depth_raw = arr
                        elif len(raw) == 0:
                            break
                    except Exception:
                        break
            self.depth_thread = threading.Thread(target=depth_reader, daemon=True, name="cam-depth-worker")
            self.depth_thread.start()
        except Exception as e:
            logger.warning("Depth pipe worker launch error: %s", e)

    def _stop_depth_pipe(self):
        self._depth_stop_evt.set()
        if self.depth_pipe_proc is not None:
            try:
                self.depth_pipe_proc.terminate()
                self.depth_pipe_proc.wait(timeout=0.5)
            except Exception:
                try:
                    self.depth_pipe_proc.kill()
                except Exception:
                    pass
            self.depth_pipe_proc = None
        if self.depth_thread is not None and self.depth_thread.is_alive():
            self.depth_thread.join(timeout=0.5)
        self.depth_thread = None

    def _open_usb(self, device_id, width, height, fps):
        if str(device_id).startswith("/dev/video"):
            if str(device_id) == "/dev/video2" and os.path.exists("/dev/video4"):
                device_id = "/dev/video4"
            else:
                device_id = self._ensure_color_node(str(device_id))
            idx: Any = int(str(device_id).replace("/dev/video", ""))
            if IS_LINUX and not os.access(str(device_id), os.R_OK | os.W_OK):
                raise RuntimeError(f"No permission for {device_id}. Run: sudo usermod -aG video $USER "
                                   "and log out/in.")
        elif str(device_id).isdigit():
            idx = int(device_id)
        elif any(k in str(device_id).lower() for k in ("realsense", "orbbec", "usb", "default")):
            # Resolve camera alias to first accessible video node
            idx = self._resolve_color_node(str(device_id))
        else:
            raise RuntimeError(f"'{device_id}' is not a USB camera id (expected index or /dev/videoN).")

        last = "no backend could open the device"
        fourcc_candidates = [
            cv2.VideoWriter_fourcc(*"MJPG"),
            cv2.VideoWriter_fourcc(*"YUYV"),
            cv2.VideoWriter_fourcc(*"Y16 "),
            0
        ]
        for be in _cv_backends():
            for fcc in fourcc_candidates:
                cap = None
                try:
                    cap = cv2.VideoCapture(idx, be)
                    if not cap.isOpened():
                        if cap is not None: cap.release()
                        continue
                    if fcc != 0:
                        cap.set(cv2.CAP_PROP_FOURCC, fcc)
                    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Low-latency live stream (drop stale kernel frames)
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
                    cap.set(cv2.CAP_PROP_FPS, fps)
                    ok = False
                    for _ in range(15):          # cameras need a few frames to warm up
                        ok, frame = cap.read()
                        if ok and frame is not None:
                            break
                        time.sleep(0.08)
                    if not ok:
                        last = "device opened but delivered no frames (in use by another app or invalid format)"
                        cap.release(); continue
                    self.cap = cap
                    self.width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)) or width
                    self.height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or height
                    with self.lock:
                        self.latest_rgb = frame

                    # Auto-start paired depth pipe on Linux if available (/dev/video2)
                    if IS_LINUX:
                        depth_node = "/dev/video2" if os.path.exists("/dev/video2") else None
                        if depth_node and os.access(depth_node, os.R_OK | os.W_OK):
                            try:
                                self._start_depth_pipe(depth_node, width=640, height=480)
                                self.has_depth_hardware = True
                            except Exception as e:
                                logger.warning("Could not start paired depth pipe: %s", e)
                    return
                except Exception as e:
                    last = str(e)
                    if cap is not None:
                        try: cap.release()
                        except Exception: pass
        raise RuntimeError(f"Could not open camera {device_id}: {last}. Close other apps using the camera "
                           "(e.g. ffplay, Zoom, browser tabs) and check USB connection.")

    def _open_network(self, url):
        url = url or "http://192.168.4.1/stream"
        cap = cv2.VideoCapture(url)
        if not cap.isOpened():
            raise RuntimeError(f"Could not connect to network stream: {url}")
        self.cap = cap

    def _open_realsense(self, device_id, width, height, fps):
        if not REALSENSE_AVAILABLE:
            raise RuntimeError("pyrealsense2 is not installed - RealSense depth needs it. "
                               "Run: pip install pyrealsense2 (Windows/Linux x86-64, Python 3.8-3.12). "
                               "On ARM Linux build librealsense from source.")
        serial = str(device_id).split(":", 1)[1] if ":" in str(device_id) else None
        ctx = rs.context()
        devs = ctx.query_devices()
        if not devs.size():
            raise RuntimeError("No RealSense device found. Use a USB 3 port/cable; on Linux install the "
                               "udev rules (99-realsense-libusb.rules).")
        
        target_dev = None
        for d in devs:
            if not serial or d.get_info(rs.camera_info.serial_number) == serial:
                target_dev = d
                break
        if target_dev is None and devs.size() > 0:
            target_dev = devs[0]

        # Try supported resolution and framerate combinations across rgb8, yuyv, bgr8
        # (Fixes "Couldn't resolve requests" on USB 2.1 ports)
        attempts = [(width, height, fps), (640, 480, 30), (640, 480, 15), (640, 360, 30), (424, 240, 15)]
        color_formats = [rs.format.rgb8, rs.format.yuyv, rs.format.bgr8]
        last = None
        
        for (w, h, f) in dict.fromkeys(attempts):
            for fmt in color_formats:
                pipe = rs.pipeline()
                cfg = rs.config()
                if target_dev:
                    cfg.enable_device(target_dev.get_info(rs.camera_info.serial_number))
                cfg.enable_stream(rs.stream.color, w, h, fmt, f)
                cfg.enable_stream(rs.stream.depth, w, h, rs.format.z16, f)
                try:
                    prof = pipe.start(cfg)
                    frames = pipe.wait_for_frames(1500)
                    if not frames:
                        pipe.stop()
                        continue
                except Exception as e:
                    last = e
                    try: pipe.stop()
                    except Exception: pass
                    continue
                self.rs_pipeline = pipe
                self._rs_color_fmt = fmt
                try:
                    self.rs_align = rs.align(rs.stream.color)
                except Exception:
                    self.rs_align = None
                try:
                    self.rs_depth_scale = prof.get_device().first_depth_sensor().get_depth_scale()
                    self._depth_scale_m = self.rs_depth_scale
                except Exception:
                    self.rs_depth_scale = 0.001
                    self._depth_scale_m = 0.001
                self.width, self.height, self.target_fps = w, h, f
                self.has_depth_hardware = True
                return

        raise RuntimeError(f"RealSense could not start RGB+Depth pipeline: {last}. Check USB connection.")

    @staticmethod
    def _reset_realsense(dev):
        """hardware_reset() then wait (<=12 s) for THE SAME serial to re-enumerate."""
        try:
            sn = dev.get_info(rs.camera_info.serial_number)
            dev.hardware_reset()
        except Exception:
            return None
        time.sleep(3.0)
        t0 = time.time()
        while time.time() - t0 < 9.0:
            try:
                for d in rs.context().query_devices():
                    if d.get_info(rs.camera_info.serial_number) == sn:
                        return d
            except Exception:
                pass
            time.sleep(0.5)
        return None

    def _open_orbbec(self, device_id, width=640, height=480, fps=30):
        if not ORBBEC_AVAILABLE:
            raise RuntimeError("pyorbbecsdk is not installed - Orbbec depth needs it. "
                               "Run: pip install pyorbbecsdk (see github.com/orbbec/pyorbbecsdk).")
        pipe = ob.Pipeline()
        cfg = ob.Config()

        def pick(sensor, fmts):
            plist = pipe.get_stream_profile_list(sensor)
            for fmt_name in fmts:
                fmt = getattr(ob.OBFormat, fmt_name, None)
                if fmt is None:
                    continue
                for f in (fps, 30, 15, 0):
                    try:
                        return plist.get_video_stream_profile(width, height, fmt, f)
                    except Exception:
                        continue
            return plist.get_default_video_stream_profile()   # last resort (can be 1080p+ = slow)

        cp = pick(ob.OBSensorType.COLOR_SENSOR, ("RGB", "BGR", "YUYV", "MJPG"))   # raw first: no JPEG decode
        dp = pick(ob.OBSensorType.DEPTH_SENSOR, ("Y16", "Y14"))
        cfg.enable_stream(cp)
        cfg.enable_stream(dp)
        try:
            cfg.set_align_mode(ob.OBAlignMode.SW_MODE)
        except Exception:
            pass
        pipe.start(cfg)
        try:                                           # prove frames arrive before reporting success
            fs = pipe.wait_for_frames(5000)
            if fs is None or fs.get_color_frame() is None:
                raise RuntimeError("Orbbec pipeline started but delivered no colour frame in 5 s "
                                   "(use a USB 3 port; on Linux install 99-obsensor-libusb.rules).")
        except Exception:
            try: pipe.stop()
            except Exception: pass
            raise
        self.ob_pipeline = pipe
        self.width, self.height = cp.get_width(), cp.get_height()
        self.has_depth_hardware = True

    # ------------------------------------------------------------------ disconnect
    def disconnect(self):
        self.is_running = False
        t = self.thread
        if t and t.is_alive() and t is not threading.current_thread():
            t.join(timeout=3.0)           # worker polls with <=500 ms timeouts
        self.thread = None
        if self.ffmpeg_capture is not None:
            try:
                self.ffmpeg_capture.stop()
            except Exception:
                pass
            self.ffmpeg_capture = None
        if self.rs_pipeline:
            try: self.rs_pipeline.stop()
            except Exception: pass
            self.rs_pipeline = None
        if self.ob_pipeline:
            try: self.ob_pipeline.stop()
            except Exception: pass
            self.ob_pipeline = None
        if self.cap:
            try: self.cap.release()
            except Exception: pass
            self.cap = None
        self._stop_depth_pipe()
        self._synthetic = False
        self.is_connected = False
        self.has_depth_hardware = False
        with self.lock:
            self.latest_rgb = None
            self.latest_depth_color = None
            self.latest_depth_raw = None
            self._jpeg_cache = (None, None)
        self.actual_fps = 0.0

    # ------------------------------------------------------------------ capture
    def _publish(self, rgb, depth_color, depth_raw=None):
        with self.lock:
            self.latest_rgb = rgb
            self.latest_depth_color = depth_color
            self.latest_depth_raw = depth_raw
            self.frame_seq += 1
        self._fps_count += 1
        now = time.time()
        if now - self._fps_t0 >= 1.0:
            self.actual_fps = round(self._fps_count / (now - self._fps_t0), 1)
            self._fps_count, self._fps_t0 = 0, now

    def _capture_worker(self):
        self._fps_count, self._fps_t0 = 0, time.time()
        first = True
        t_syn = 0.0
        last_ffmpeg_seq = -1
        while self.is_running:
            try:
                if self.ffmpeg_capture is not None:
                    with self.ffmpeg_capture.lock:
                        cur_seq = self.ffmpeg_capture.frame_seq
                        rgb = self.ffmpeg_capture.latest_rgb
                        depth_raw = self.ffmpeg_capture.latest_depth_raw
                    if rgb is not None and cur_seq != last_ffmpeg_seq:
                        last_ffmpeg_seq = cur_seq
                        self._publish(rgb, None, depth_raw)
                    time.sleep(0.015)
                    continue
                elif self.rs_pipeline:
                    frames = self.rs_pipeline.wait_for_frames(2000 if first else 500)
                    first = False
                    if self.rs_align:
                        try:
                            frames = self.rs_align.process(frames)
                        except Exception:
                            pass
                    c = frames.get_color_frame()
                    d = frames.get_depth_frame()
                    if d:
                        self._cached_depth_raw = np.asanyarray(d.get_data()).copy()
                    if c:
                        raw_rgb = np.asanyarray(c.get_data()).copy()
                        fmt = getattr(self, "_rs_color_fmt", None)
                        if fmt == rs.format.rgb8:
                            rgb = cv2.cvtColor(raw_rgb, cv2.COLOR_RGB2BGR)
                        elif fmt == rs.format.yuyv:
                            rgb = cv2.cvtColor(raw_rgb, cv2.COLOR_YUV2BGR_YUY2)
                        else:
                            rgb = raw_rgb
                        self._publish(rgb, None, self._cached_depth_raw)
                    elif d and self.latest_rgb is not None:
                        self._publish(self.latest_rgb, None, self._cached_depth_raw)
                elif self.ob_pipeline:
                    frames = self.ob_pipeline.wait_for_frames(500)
                    if frames is None:
                        self.dropped_frames += 1; continue
                    c = frames.get_color_frame()
                    d = frames.get_depth_frame()
                    if d:
                        try:
                            raw = np.frombuffer(d.get_data(), dtype=np.uint16).reshape(d.get_height(), d.get_width()).copy()
                            self._depth_scale_m = (d.get_depth_scale() if hasattr(d, "get_depth_scale") else 1.0) / 1000.0
                            self._cached_depth_raw = raw
                        except Exception:
                            pass
                    if c:
                        rgb = self._ob_color_to_bgr(c)
                        if rgb is not None:
                            raw_depth = self._cached_depth_raw
                            if raw_depth is not None and raw_depth.shape[:2] != rgb.shape[:2]:
                                raw_depth = cv2.resize(raw_depth, (rgb.shape[1], rgb.shape[0]), interpolation=cv2.INTER_NEAREST)
                            self._publish(rgb, None, raw_depth)
                    elif d and self.latest_rgb is not None:
                        self._publish(self.latest_rgb, None, self._cached_depth_raw)
                elif self._synthetic:
                    t_syn += 1.0 / max(1, self.target_fps)
                    rgb, raw = _synthetic_frame(self.width, self.height, t_syn)
                    self._depth_scale_m = 0.001
                    self._publish(rgb, None, raw)
                    time.sleep(1.0 / max(1, self.target_fps))
                elif self.cap is not None:
                    ok, frame = self.cap.read()
                    if not ok or frame is None:
                        self.dropped_frames += 1
                        self.last_error = "camera stopped delivering frames (unplugged or claimed by another app)"
                        time.sleep(0.05); continue

                    # Remove bottom sensor register scanlines (fixes horizontal lines on RealSense V4L2)
                    hf, wf = frame.shape[:2]
                    if hf > 100:
                        frame = cv2.resize(frame[:hf - 6, :], (wf, hf), interpolation=cv2.INTER_LINEAR)

                    raw_depth = self._cached_depth_raw
                    if raw_depth is not None and raw_depth.shape[:2] != frame.shape[:2]:
                        try:
                            raw_depth = cv2.resize(raw_depth, (frame.shape[1], frame.shape[0]), interpolation=cv2.INTER_NEAREST)
                        except Exception:
                            pass
                    self._publish(_as_bgr8(frame), None, raw_depth)
                else:
                    break
            except Exception as e:
                msg = str(e)
                if "Frame didn't arrive" in msg or "timeout" in msg.lower():
                    self.dropped_frames += 1
                else:
                    self.last_error = msg
                time.sleep(0.02)

    @staticmethod
    def _ob_color_to_bgr(frame):
        data = np.asanyarray(frame.get_data())
        fmt = frame.get_format()
        w, h = frame.get_width(), frame.get_height()
        if fmt == ob.OBFormat.MJPG:
            return cv2.imdecode(data, cv2.IMREAD_COLOR)
        if fmt == ob.OBFormat.RGB:
            return cv2.cvtColor(data.reshape(h, w, 3), cv2.COLOR_RGB2BGR)
        if fmt == ob.OBFormat.BGR:
            return data.reshape(h, w, 3).copy()
        if fmt == ob.OBFormat.YUYV:
            return cv2.cvtColor(data.reshape(h, w, 2), cv2.COLOR_YUV2BGR_YUY2)
        if fmt == ob.OBFormat.UYVY:
            return cv2.cvtColor(data.reshape(h, w, 2), cv2.COLOR_YUV2BGR_UYVY)
        raise RuntimeError(f"Unsupported Orbbec colour format: {fmt}")

    # ------------------------------------------------------------------ output
    def get_jpeg_frame(self, mode: str = "rgb") -> Optional[bytes]:
        if not CV2_AVAILABLE:
            return None
        if mode not in STREAM_MODES:
            mode = "rgb"
        with self.lock:
            if self.latest_rgb is None:
                return None
            key = (self.frame_seq, mode, self.brightness, self.contrast)
            if self._jpeg_cache[0] == key:
                return self._jpeg_cache[1]
            rgb = self.latest_rgb
            raw_depth = self.latest_depth_raw
            scale_m = self._depth_scale_m
        frame = _as_bgr8(rgb)
        if self.brightness != 50 or self.contrast != 50:
            frame = cv2.convertScaleAbs(frame, alpha=self.contrast / 50.0, beta=(self.brightness - 50) * 1.5)

        def labelled_depth():
            # Built ONLY on demand (depth / side_by_side) - never per captured frame.
            if raw_depth is not None:
                d = colorize_depth(raw_depth, scale_m)
            else:
                d = pseudo_depth(frame)
                cv2.putText(d, "PSEUDO-DEPTH (luminance, not metric)", (10, 24),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
            if d.shape[:2] != frame.shape[:2]:
                d = cv2.resize(d, (frame.shape[1], frame.shape[0]), interpolation=cv2.INTER_NEAREST)
            return d

        if mode == "depth":
            out = labelled_depth()
        elif mode == "side_by_side":
            out = np.hstack([frame, labelled_depth()])    # always exactly 2x width
        else:
            out = frame
        ok, buf = cv2.imencode(".jpg", out, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
        data = buf.tobytes() if ok else None
        with self.lock:
            self._jpeg_cache = (key, data)
        return data

    def get_placeholder_jpeg(self, text: str = "Camera Standby") -> Optional[bytes]:
        if not CV2_AVAILABLE:
            return None
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        img[:] = (12, 17, 30)
        cv2.putText(img, text, (120, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (200, 220, 255), 2)
        ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        return buf.tobytes() if ok else None

    def get_frame(self):
        """Return the latest RGB numpy array, or None."""
        with self.lock:
            return self.latest_rgb.copy() if self.latest_rgb is not None else None

    def get_status(self) -> Dict[str, Any]:
        return {
            "connected": self.is_connected, "camera_type": self.camera_type, "device_id": self.device_id,
            "width": self.width, "height": self.height, "target_fps": self.target_fps,
            "actual_fps": self.actual_fps, "has_depth": self.has_depth_hardware,
            "depth_is_real": self.has_depth_hardware, "stream_mode": self.stream_mode,
            "dropped_frames": self.dropped_frames, "brightness": self.brightness,
            "contrast": self.contrast, "exposure": self.exposure, "last_error": self.last_error,
            "frame_seq": self.frame_seq,
        }


def _synthetic_frame(w: int, h: int, t: float):
    """Moving gradient RGB + planar/spherical metric depth (mm) for pipeline tests."""
    yy, xx = np.mgrid[0:h, 0:w]
    rgb = np.zeros((h, w, 3), np.uint8)
    rgb[..., 0] = (xx * 255 // max(1, w - 1)).astype(np.uint8)
    rgb[..., 1] = (yy * 255 // max(1, h - 1)).astype(np.uint8)
    rgb[..., 2] = int(127 + 127 * np.sin(t))
    cx, cy = int(w / 2 + (w / 4) * np.sin(t)), h // 2
    depth = (500 + xx * 2500 // max(1, w - 1)).astype(np.uint16)          # 0.5 m .. 3 m ramp
    mask = (xx - cx) ** 2 + (yy - cy) ** 2 < (min(w, h) // 5) ** 2
    depth[mask] = 400                                                       # near blob
    depth[:8, :8] = 0                                                       # invalid corner
    return rgb, depth
