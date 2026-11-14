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
    Invalid pixels (0) are black."""
    meters = depth_u16.astype(np.float32) * float(depth_scale)
    norm = np.clip((meters - min_m) / max(1e-6, (max_m - min_m)), 0.0, 1.0)
    img8 = (255 - (norm * 255)).astype(np.uint8)  # near = warm
    out = cv2.applyColorMap(img8, cv2.COLORMAP_JET)
    out[depth_u16 == 0] = 0
    return out


def pseudo_depth(bgr):
    """Luminance colour-map. NOT metric depth - always watermarked by the caller."""
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    return cv2.applyColorMap(gray, cv2.COLORMAP_TURBO)


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
        self.rs_pipeline = None
        self.rs_align = None
        self.rs_depth_scale = 0.001
        self.ob_pipeline = None
        self.ob_depth_scale = 1.0
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

    def _linux_video_devices(self) -> List[Dict[str, Any]]:
        out = []
        for p in sorted(glob.glob("/dev/video*")):
            try:
                idx = int(p.replace("/dev/video", ""))
            except ValueError:
                continue
            sys_dir = f"/sys/class/video4linux/video{idx}"
            name = f"Video device {idx}"
            try:
                with open(os.path.join(sys_dir, "name")) as f:
                    name = f.read().strip() or name
                # index 0 == the capture node; >0 are metadata / IR / secondary nodes
                with open(os.path.join(sys_dir, "index")) as f:
                    if int(f.read().strip() or 0) != 0:
                        continue
            except Exception:
                pass
            low = name.lower()
            if "realsense" in low or "orbbec" in low:
                # handled by their own SDK entries; the UVC node would give RGB only
                name = f"{name} (UVC colour only)"
            out.append({"id": p, "name": f"{name} ({p})", "type": "usb",
                        "has_rgb": True, "has_depth": False, "default_res": "640x480",
                        "accessible": os.access(p, os.R_OK | os.W_OK)})
        return out

    def _probe_indices(self, max_idx: int = 4) -> List[Dict[str, Any]]:
        out = []
        for idx in range(max_idx):
            if self.is_connected and self.camera_type == "usb" and str(idx) == str(self.device_id):
                out.append(self._usb_entry(idx))  # don't re-open the live camera
                continue
            ok = False
            for be in _cv_backends()[:1]:
                try:
                    c = cv2.VideoCapture(idx, be)
                    ok = c.isOpened()
                    c.release()
                except Exception:
                    ok = False
            if ok:
                out.append(self._usb_entry(idx))
        return out

    @staticmethod
    def _usb_entry(idx: int) -> Dict[str, Any]:
        return {"id": str(idx), "name": f"Camera {idx}", "type": "usb", "has_rgb": True,
                "has_depth": False, "default_res": "640x480"}

    def list_devices(self, force: bool = False) -> List[Dict[str, Any]]:
        now = time.time()
        if not force and now - self._enum_cache[0] < 5.0 and self._enum_cache[1]:
            return list(self._enum_cache[1])

        devices: List[Dict[str, Any]] = []

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
                devices += self._linux_video_devices() if IS_LINUX else self._probe_indices()
            except Exception as e:
                self.last_error = f"USB enumeration failed: {e}"

        if os.environ.get("SO101_TEST_CAMERA") == "1":
            devices.append({"id": "synthetic", "name": "Synthetic Test Camera (RGB + metric depth)",
                            "type": "synthetic", "has_rgb": True, "has_depth": True,
                            "default_res": "640x480"})

        devices.append({"id": "network_esp32", "name": "ESP32-CAM / IP Stream", "type": "network",
                        "has_rgb": True, "has_depth": False, "default_res": "640x480"})
        self._enum_cache = (now, devices)
        return list(devices)

    # ------------------------------------------------------------------ connect
    def connect(self, device_id: str, camera_type: str = "usb", width: int = 640, height: int = 480,
                fps: int = 30, custom_url: Optional[str] = None) -> Dict[str, Any]:
        with self._connect_lock:
            self.disconnect()
            self.device_id, self.camera_type = str(device_id), camera_type
            self.width, self.height, self.target_fps = int(width), int(height), int(fps)
            self.has_depth_hardware = False
            self.last_error = ""
            self.dropped_frames = 0

            if camera_type != "synthetic" and not CV2_AVAILABLE:
                return self._fail(f"OpenCV is not usable ({CV2_ERROR or 'not installed'}). "
                                  "Run: pip install opencv-python numpy")
            try:
                if camera_type == "realsense":
                    self._open_realsense(device_id, width, height, fps)
                elif camera_type == "orbbec":
                    self._open_orbbec(device_id)
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
                    "message": f"Connected to {device_id} ({self.width}x{self.height} @ {self.target_fps} FPS)",
                    "camera_type": camera_type, "has_depth": self.has_depth_hardware,
                    "depth_is_real": self.has_depth_hardware}

    def _fail(self, msg: str) -> Dict[str, Any]:
        self.last_error = msg
        return {"success": False, "message": msg}

    def _open_usb(self, device_id, width, height, fps):
        if str(device_id).startswith("/dev/video"):
            idx: Any = int(str(device_id).replace("/dev/video", ""))
            if IS_LINUX and not os.access(str(device_id), os.R_OK | os.W_OK):
                raise RuntimeError(f"No permission for {device_id}. Run: sudo usermod -aG video $USER "
                                   "and log out/in.")
        elif str(device_id).isdigit():
            idx = int(device_id)
        else:
            raise RuntimeError(f"'{device_id}' is not a USB camera id (expected index or /dev/videoN).")

        last = "no backend could open the device"
        for be in _cv_backends():
            cap = None
            try:
                cap = cv2.VideoCapture(idx, be)
                if not cap.isOpened():
                    cap.release(); continue
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
                cap.set(cv2.CAP_PROP_FPS, fps)
                ok = False
                for _ in range(20):          # cameras need a few frames to warm up
                    ok, frame = cap.read()
                    if ok and frame is not None:
                        break
                    time.sleep(0.1)
                if not ok:
                    last = "device opened but delivered no frames (in use by another app?)"
                    cap.release(); continue
                self.cap = cap
                self.width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)) or width
                self.height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or height
                with self.lock:
                    self.latest_rgb = frame
                return
            except Exception as e:
                last = str(e)
                if cap is not None:
                    try: cap.release()
                    except Exception: pass
        raise RuntimeError(f"Could not open camera {device_id}: {last}. Close other apps using the camera "
                           "(Zoom/Teams/browser tab) and check OS camera privacy settings.")

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
        if not rs.context().query_devices().size():
            raise RuntimeError("No RealSense device found. Use a USB 3 port/cable; on Linux install the "
                               "udev rules (99-realsense-libusb.rules).")
        # requested -> common supported pairs (D415/D435/D455/D405/L515 all do 640x480@30)
        attempts = [(width, height, fps), (640, 480, 30), (848, 480, 30), (1280, 720, 30), (640, 360, 30)]
        last = None
        for (w, h, f) in dict.fromkeys(attempts):
            pipe = rs.pipeline()
            cfg = rs.config()
            if serial:
                cfg.enable_device(serial)
            cfg.enable_stream(rs.stream.color, w, h, rs.format.bgr8, f)
            cfg.enable_stream(rs.stream.depth, w, h, rs.format.z16, f)
            try:
                prof = pipe.start(cfg)
            except Exception as e:
                last = e; continue
            self.rs_pipeline = pipe
            self.rs_align = rs.align(rs.stream.color)
            try:
                self.rs_depth_scale = prof.get_device().first_depth_sensor().get_depth_scale()
            except Exception:
                self.rs_depth_scale = 0.001
            self.width, self.height, self.target_fps = w, h, f
            self.has_depth_hardware = True
            return
        raise RuntimeError(f"RealSense could not start any RGB+Depth profile: {last}")

    def _open_orbbec(self, device_id):
        if not ORBBEC_AVAILABLE:
            raise RuntimeError("pyorbbecsdk is not installed - Orbbec depth needs it. "
                               "Run: pip install pyorbbecsdk (see github.com/orbbec/pyorbbecsdk).")
        pipe = ob.Pipeline()
        cfg = ob.Config()
        cp = pipe.get_stream_profile_list(ob.OBSensorType.COLOR_SENSOR).get_default_video_stream_profile()
        dp = pipe.get_stream_profile_list(ob.OBSensorType.DEPTH_SENSOR).get_default_video_stream_profile()
        cfg.enable_stream(cp)
        cfg.enable_stream(dp)
        try:
            cfg.set_align_mode(ob.OBAlignMode.SW_MODE)
        except Exception:
            pass
        pipe.start(cfg)
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
        while self.is_running:
            try:
                if self.rs_pipeline:
                    frames = self.rs_pipeline.wait_for_frames(5000 if first else 500)
                    first = False
                    frames = self.rs_align.process(frames)
                    c, d = frames.get_color_frame(), frames.get_depth_frame()
                    if not c or not d:
                        self.dropped_frames += 1; continue
                    raw = np.asanyarray(d.get_data()).copy()
                    self._publish(np.asanyarray(c.get_data()).copy(),
                                  colorize_depth(raw, self.rs_depth_scale), raw)
                elif self.ob_pipeline:
                    frames = self.ob_pipeline.wait_for_frames(500)
                    if frames is None:
                        self.dropped_frames += 1; continue
                    c, d = frames.get_color_frame(), frames.get_depth_frame()
                    if c is None or d is None:
                        self.dropped_frames += 1; continue
                    rgb = self._ob_color_to_bgr(c)
                    raw = np.frombuffer(d.get_data(), dtype=np.uint16).reshape(d.get_height(), d.get_width()).copy()
                    scale = d.get_depth_scale() if hasattr(d, "get_depth_scale") else 1.0
                    if raw.shape[:2] != rgb.shape[:2]:
                        raw = cv2.resize(raw, (rgb.shape[1], rgb.shape[0]), interpolation=cv2.INTER_NEAREST)
                    self._publish(rgb, colorize_depth(raw, scale), raw)
                elif self._synthetic:
                    t_syn += 1.0 / max(1, self.target_fps)
                    rgb, raw = _synthetic_frame(self.width, self.height, t_syn)
                    self._publish(rgb, colorize_depth(raw, 0.001), raw)
                    time.sleep(1.0 / max(1, self.target_fps))
                elif self.cap is not None:
                    ok, frame = self.cap.read()
                    if not ok or frame is None:
                        self.dropped_frames += 1
                        self.last_error = "camera stopped delivering frames (unplugged or claimed by another app)"
                        time.sleep(0.05); continue
                    self._publish(frame, pseudo_depth(frame), None)
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
            depth = self.latest_depth_color
            real = self.has_depth_hardware
        frame = rgb
        if self.brightness != 50 or self.contrast != 50:
            frame = cv2.convertScaleAbs(rgb, alpha=self.contrast / 50.0, beta=(self.brightness - 50) * 1.5)

        def labelled_depth():
            d = depth if depth is not None else pseudo_depth(frame)
            d = d.copy()
            if not real:
                cv2.putText(d, "PSEUDO-DEPTH (luminance, not metric)", (10, 24),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
            return d

        if mode == "depth":
            out = labelled_depth()
        elif mode == "side_by_side":
            d = labelled_depth()
            if d.shape[:2] != frame.shape[:2]:
                d = cv2.resize(d, (frame.shape[1], frame.shape[0]))
            out = np.hstack([frame, d])
            if out.shape[1] > 1280:      # keep bandwidth sane on slow links
                s = 1280.0 / out.shape[1]
                out = cv2.resize(out, (1280, int(out.shape[0] * s)))
        else:
            out = frame
        ok, buf = cv2.imencode(".jpg", out, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
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
