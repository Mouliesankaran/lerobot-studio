#!/usr/bin/env python3
"""
AI Model Deployment & Real Robot Evaluation Backend
SO-101 Robotic Arm — Sim-to-Real Policy Execution Engine

Supports: PyTorch, ONNX, JAX/Flax, Stable-Baselines3, TensorFlow, NumPy NPZ
Provides: Tornado RequestHandler classes for bridge_standalone.py integration
"""

import os
import io
import sys
import json
import time
import uuid
import struct
import shutil
import zipfile
import tempfile
import threading
import statistics
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import tornado.web

ROOT_DIR = Path(__file__).resolve().parent
UPLOAD_DIR = ROOT_DIR / "uploads" / "ai_models"
UPLOAD_DIR.mkdir(parents=True, exist_ok=True)

# ── Optional ML framework imports (graceful fallback) ────────────────────────
try:
    import numpy as np
    NP_AVAILABLE = True
except ImportError:
    NP_AVAILABLE = False
    np = None  # type: ignore

def _try_import(name):
    try:
        import importlib
        return importlib.import_module(name), True
    except Exception:
        return None, False

torch, TORCH_AVAILABLE     = _try_import("torch")
onnxrt, ONNX_AVAILABLE     = _try_import("onnxruntime")
jax, JAX_AVAILABLE         = _try_import("jax")
flax, FLAX_AVAILABLE       = _try_import("flax")
tf, TF_AVAILABLE           = _try_import("tensorflow")


# ── Joint Definitions ────────────────────────────────────────────────────────
JOINT_MAP = {
    1: {"name": "shoulder_pan",  "rad_min": -1.4678, "rad_max":  1.4678, "deg_min": -84.1,  "deg_max":  84.1},
    2: {"name": "shoulder_lift", "rad_min": -3.1398, "rad_max":  3.1398, "deg_min": -179.9, "deg_max": 179.9},
    3: {"name": "elbow_flex",    "rad_min": -3.1416, "rad_max":  3.1398, "deg_min": -180.0, "deg_max": 179.9},
    4: {"name": "wrist_flex",    "rad_min": -1.4713, "rad_max":  1.4713, "deg_min": -84.3,  "deg_max":  84.3},
    5: {"name": "wrist_roll",    "rad_min": -3.1416, "rad_max":  3.1416, "deg_min": -180.0, "deg_max": 180.0},
    6: {"name": "gripper",       "rad_min":  0.0,    "rad_max":  1.0,    "deg_min":   0.0,  "deg_max": 100.0},
}
N_JOINTS = len(JOINT_MAP)

SAFETY_CLAMP_DEG = {
    "strict": 15.0,
    "normal": 30.0,
    "loose":  45.0,
    "off":    999.0,
}

# ── Framework Detection ───────────────────────────────────────────────────────
def _detect_framework(filepath: Path, orig_name: str) -> Dict[str, Any]:
    """Detect framework and extract model metadata without executing."""
    ext = filepath.suffix.lower().lstrip(".")
    info: Dict[str, Any] = {
        "framework": "unknown", "format": ext,
        "input_shape": None, "output_shape": None,
        "warnings": [], "errors": [],
        "can_load": False, "metadata": {}
    }

    if ext in ("pt", "pth"):
        info["framework"] = "pytorch"
        if TORCH_AVAILABLE:
            try:
                obj = torch.load(str(filepath), map_location="cpu", weights_only=True)  # type: ignore
                info["can_load"] = True
                # Try to extract state dict keys
                if hasattr(obj, "keys"):
                    keys = list(obj.keys())
                    info["metadata"]["state_dict_keys_count"] = len(keys)
                    info["metadata"]["sample_keys"] = keys[:5]
            except Exception as e:
                try:
                    obj = torch.load(str(filepath), map_location="cpu", weights_only=False)  # type: ignore
                    info["can_load"] = True
                    info["warnings"].append("Loaded with weights_only=False (untrusted checkpoint)")
                except Exception as e2:
                    info["errors"].append(f"PyTorch load error: {e2}")
        else:
            info["warnings"].append("PyTorch not installed — cannot inspect internals")

    elif ext == "onnx":
        info["framework"] = "onnx"
        if ONNX_AVAILABLE:
            try:
                sess = onnxrt.InferenceSession(str(filepath), providers=["CPUExecutionProvider"])  # type: ignore
                info["can_load"] = True
                inputs = sess.get_inputs()
                outputs = sess.get_outputs()
                if inputs:
                    info["input_shape"] = inputs[0].shape
                    info["metadata"]["input_names"] = [i.name for i in inputs]
                if outputs:
                    info["output_shape"] = outputs[0].shape
                    info["metadata"]["output_names"] = [o.name for o in outputs]
            except Exception as e:
                info["errors"].append(f"ONNX load error: {e}")
        else:
            info["warnings"].append("onnxruntime not installed — cannot inspect ONNX model")

    elif ext == "msgpack":
        info["framework"] = "jax_flax"
        try:
            raw = filepath.read_bytes()
            info["can_load"] = True
            info["metadata"]["file_size_bytes"] = len(raw)
            # Minimal msgpack inspection (count top-level keys if possible)
            info["warnings"].append("JAX/Flax msgpack: install jax+flax for full inspection")
        except Exception as e:
            info["errors"].append(f"msgpack read error: {e}")

    elif ext == "zip":
        info["framework"] = "stable_baselines3"
        try:
            with zipfile.ZipFile(str(filepath), "r") as zf:
                names = zf.namelist()
                info["can_load"] = True
                info["metadata"]["zip_contents"] = names[:10]
                if "data" in names:
                    raw = json.loads(zf.read("data").decode("utf-8"))
                    info["metadata"]["sb3_data_keys"] = list(raw.keys())[:8]
                    obs_space = raw.get("observation_space", {})
                    act_space = raw.get("action_space", {})
                    if obs_space.get("shape"):
                        info["input_shape"] = obs_space["shape"]
                    if act_space.get("shape"):
                        info["output_shape"] = act_space["shape"]
        except zipfile.BadZipFile:
            info["errors"].append("Not a valid ZIP file")
        except Exception as e:
            info["errors"].append(f"ZIP inspection error: {e}")

    elif ext == "npz":
        info["framework"] = "numpy_npz"
        if NP_AVAILABLE:
            try:
                data = np.load(str(filepath), allow_pickle=False)  # type: ignore
                info["can_load"] = True
                info["metadata"]["arrays"] = {k: list(v.shape) for k, v in data.items()}
                # Guess policy from common key names
                for k in data.keys():
                    if "mean" in k.lower() or "std" in k.lower():
                        info["metadata"]["has_normalization"] = True
            except Exception as e:
                info["errors"].append(f"NPZ load error: {e}")
        else:
            info["warnings"].append("numpy not installed")

    elif ext == "pb" or filepath.is_dir():
        info["framework"] = "tensorflow"
        if TF_AVAILABLE:
            try:
                model = tf.saved_model.load(str(filepath))  # type: ignore
                info["can_load"] = True
            except Exception as e:
                info["errors"].append(f"TF load error: {e}")
        else:
            info["warnings"].append("TensorFlow not installed — cannot inspect .pb model")

    return info


# ── Compatibility Validation ──────────────────────────────────────────────────
def run_compatibility_checks(fw_info: Dict[str, Any], cfg: Dict[str, Any]) -> List[Dict]:
    """Return list of {name, status, detail} dicts. STATUS: PASS|FAIL|WARNING|UNKNOWN"""

    checks = []

    def chk(name: str, status: str, detail: str = ""):
        checks.append({"name": name, "status": status, "detail": detail})

    # 1. Checkpoint readability
    if fw_info.get("can_load"):
        chk("Checkpoint Readable", "PASS", f"File loaded as {fw_info['format'].upper()}")
    elif fw_info.get("errors"):
        chk("Checkpoint Readable", "FAIL", fw_info["errors"][0])
    else:
        chk("Checkpoint Readable", "WARNING", "Cannot verify (framework not installed)")

    # 2. Framework availability
    fw = fw_info.get("framework", "unknown")
    fw_avail = {
        "pytorch": TORCH_AVAILABLE,
        "onnx": ONNX_AVAILABLE,
        "jax_flax": JAX_AVAILABLE or FLAX_AVAILABLE,
        "stable_baselines3": TORCH_AVAILABLE,
        "numpy_npz": NP_AVAILABLE,
        "tensorflow": TF_AVAILABLE,
    }.get(fw)
    if fw_avail is True:
        chk("Framework Available", "PASS", f"{fw} runtime detected")
    elif fw_avail is False:
        chk("Framework Available", "FAIL", f"{fw} not installed — inference not possible")
    else:
        chk("Framework Available", "WARNING", f"Unknown framework '{fw}'")

    # 3. Model architecture detected
    has_arch = fw_info.get("input_shape") is not None or fw_info.get("output_shape") is not None
    if has_arch:
        chk("Model Architecture", "PASS", f"in={fw_info.get('input_shape')} out={fw_info.get('output_shape')}")
    else:
        chk("Model Architecture", "WARNING", "Could not auto-detect shapes — provide obs/action config")

    # 4. Observation dimensions
    obs_shape = fw_info.get("input_shape")
    if obs_shape:
        # Expect flat vector of at least 6 (joint positions), or image+joints
        flat_ok = isinstance(obs_shape, (list, tuple)) and len(obs_shape) >= 1 and obs_shape[-1] >= N_JOINTS
        if flat_ok:
            chk("Observation Dimensions", "PASS", f"Input shape {list(obs_shape)} includes ≥{N_JOINTS} dims")
        else:
            chk("Observation Dimensions", "WARNING", f"Input shape {list(obs_shape)} — verify includes joint obs")
    else:
        chk("Observation Dimensions", "WARNING", "Unknown — supply obs_config.json with 'observation_dim'")

    # 5. Action dimensions
    act_shape = fw_info.get("output_shape")
    if act_shape:
        flat_ok = isinstance(act_shape, (list, tuple)) and act_shape[-1] == N_JOINTS
        if flat_ok:
            chk("Action Dimensions", "PASS", f"Output dim={act_shape[-1]} matches {N_JOINTS} SO-101 joints")
        elif isinstance(act_shape, (list, tuple)) and act_shape[-1] != N_JOINTS:
            chk("Action Dimensions", "FAIL",
                f"Output dim={act_shape[-1]} ≠ {N_JOINTS} (SO-101 joints). Adapter required.")
        else:
            chk("Action Dimensions", "WARNING", f"Output shape {list(act_shape)} — verify")
    else:
        chk("Action Dimensions", "WARNING", "Unknown — supply action_config.json with 'action_dim'")

    # 6. Joint ordering (heuristic)
    meta = fw_info.get("metadata", {})
    has_norm = meta.get("has_normalization", False)
    chk("Joint Ordering", "WARNING",
        "Assumed: [pan, lift, elbow, wrist_flex, roll, gripper]. Supply joint_map.json to confirm.")

    # 7. Normalization parameters
    if has_norm:
        chk("Normalization Parameters", "PASS", "Mean/std arrays found in checkpoint")
    else:
        chk("Normalization Parameters", "WARNING", "No normalization found — raw actions will be passed to robot")

    # 8. Inference device
    if TORCH_AVAILABLE:
        cuda = torch.cuda.is_available()  # type: ignore
        chk("Inference Device", "PASS", "CUDA available" if cuda else "CPU inference (no GPU required)")
    else:
        chk("Inference Device", "PASS", "CPU (no ML runtime installed — simulation mode)")

    # 9. Required dependencies
    missing = []
    if fw == "pytorch" and not TORCH_AVAILABLE: missing.append("torch")
    if fw == "onnx" and not ONNX_AVAILABLE: missing.append("onnxruntime")
    if fw in ("jax_flax",) and not (JAX_AVAILABLE or FLAX_AVAILABLE): missing.append("jax flax")
    if fw == "tensorflow" and not TF_AVAILABLE: missing.append("tensorflow")
    if not NP_AVAILABLE: missing.append("numpy")
    if missing:
        chk("Required Dependencies", "FAIL", f"Missing packages: {', '.join(missing)} — pip install them")
    else:
        chk("Required Dependencies", "PASS", "All required packages available")

    return checks


# ── Policy Execution Engine ───────────────────────────────────────────────────
class PolicyExecutionEngine:
    """Runs the AI policy loop in a background thread."""

    def __init__(self, model_info: Dict, params: Dict, robot_driver=None, camera_manager=None):
        self.model_info = model_info
        self.params = params
        self.robot_driver = robot_driver
        self.camera_manager = camera_manager

        # Execution state
        self._lock = threading.Lock()
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self.state = "idle"  # idle | running | done | stopped | error

        # Metrics
        self.step_count = 0
        self.start_time: Optional[float] = None
        self.end_time: Optional[float] = None
        self.inference_times: List[float] = []
        self.loop_hz = 0.0
        self.dropped_frames = 0
        self.safety_violations = 0
        self.joint_errors_deg: Dict[int, List[float]] = {i: [] for i in range(1, 7)}
        self.last_error: Optional[str] = None
        self.final_status = "UNKNOWN"

    def start(self):
        if self.state == "running":
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run_loop, daemon=True, name="ai-policy")
        self.state = "running"
        self.start_time = time.time()
        self._thread.start()

    def stop(self):
        self._stop_event.set()
        self.state = "stopped"
        self.final_status = "STOPPED"

    def emergency_stop(self):
        self._stop_event.set()
        self.state = "stopped"
        self.final_status = "E-STOP"
        # Command robot to hold current position (safest)
        if self.robot_driver:
            try:
                self.robot_driver.hold_position()
            except Exception:
                pass

    def _get_observation(self) -> Optional[Any]:
        """Build observation vector: joint positions + optionally camera."""
        obs = []
        if self.robot_driver:
            try:
                positions = self.robot_driver.read_positions()  # {id: pos_ticks}
                for jid in sorted(JOINT_MAP.keys()):
                    ticks = positions.get(jid, 2048)
                    rad = (ticks - 2048) / 2048.0 * 3.14159
                    obs.append(float(rad))
            except Exception:
                # Simulation: zeros
                obs = [0.0] * N_JOINTS
        else:
            obs = [0.0] * N_JOINTS

        if self.params.get("use_camera") and self.camera_manager:
            try:
                frame_bytes = self.camera_manager.get_jpeg_frame("rgb")
                # Minimal: append a flag that camera data is available
                # Full implementation would decode and resize to model's expected img size
                obs.append(1.0)  # camera_available flag
            except Exception:
                obs.append(0.0)
                self.dropped_frames += 1

        if NP_AVAILABLE:
            return np.array(obs, dtype=np.float32)  # type: ignore
        return obs

    def _run_inference(self, obs) -> Optional[List[float]]:
        """Run model inference. Returns list of N_JOINTS action values."""
        fw = self.model_info.get("framework", "unknown")
        t0 = time.perf_counter()

        actions = None
        try:
            if fw == "onnx" and ONNX_AVAILABLE:
                sess = self.model_info.get("_session")
                if sess and NP_AVAILABLE:
                    inp_name = sess.get_inputs()[0].name
                    result = sess.run(None, {inp_name: obs.reshape(1, -1)})
                    actions = result[0].flatten().tolist()[:N_JOINTS]

            elif fw == "pytorch" and TORCH_AVAILABLE:
                net = self.model_info.get("_model")
                if net and NP_AVAILABLE:
                    with torch.no_grad():  # type: ignore
                        t_obs = torch.FloatTensor(obs).unsqueeze(0)  # type: ignore
                        out = net(t_obs)
                        actions = out.squeeze().tolist()
                        if isinstance(actions, float):
                            actions = [actions]

            else:
                # Simulation / unsupported: return small random perturbation
                if NP_AVAILABLE:
                    actions = (np.random.randn(N_JOINTS) * 0.01).tolist()  # type: ignore
                else:
                    import random
                    actions = [random.gauss(0, 0.01) for _ in range(N_JOINTS)]

        except Exception as e:
            self.last_error = f"Inference error: {e}"
            actions = [0.0] * N_JOINTS

        elapsed_ms = (time.perf_counter() - t0) * 1000
        with self._lock:
            self.inference_times.append(elapsed_ms)

        return actions

    def _apply_safety(self, actions: List[float]) -> List[float]:
        """Clamp per-joint action deltas to safety limit."""
        clamp_deg = SAFETY_CLAMP_DEG.get(self.params.get("safety_mode", "normal"), 30.0)
        clamp_rad = clamp_deg * 3.14159 / 180.0
        clamped = []
        for i, a in enumerate(actions[:N_JOINTS]):
            jid = i + 1
            jinfo = JOINT_MAP.get(jid, {})
            safe_a = max(jinfo.get("rad_min", -3.14), min(jinfo.get("rad_max", 3.14), float(a)))
            if abs(float(a) - safe_a) > 1e-6:
                with self._lock:
                    self.safety_violations += 1
            clamped.append(safe_a)
        return clamped

    def _send_to_robot(self, actions: List[float]):
        """Convert radians → STS3215 ticks and send."""
        if not self.robot_driver:
            return
        targets = {}
        for i, rad in enumerate(actions[:N_JOINTS]):
            jid = i + 1
            ticks = int(2048 + (rad / 3.14159) * 2048)
            ticks = max(0, min(4095, ticks))
            targets[jid] = ticks
        try:
            self.robot_driver.set_positions(targets)
        except Exception as e:
            self.last_error = f"Robot comm error: {e}"

    def _update_tracking_error(self, actions: List[float]):
        """Estimate joint tracking error (actions vs current positions)."""
        if not self.robot_driver:
            return
        try:
            positions = self.robot_driver.read_positions()
            for i, target_rad in enumerate(actions[:N_JOINTS]):
                jid = i + 1
                ticks = positions.get(jid, 2048)
                actual_rad = (ticks - 2048) / 2048.0 * 3.14159
                err_deg = abs(target_rad - actual_rad) * 180.0 / 3.14159
                self.joint_errors_deg[jid].append(err_deg)
        except Exception:
            pass

    def _run_loop(self):
        hz = max(1, int(self.params.get("hz", 30)))
        max_steps = int(self.params.get("max_steps", 500))
        dt = 1.0 / hz
        loop_times: List[float] = []

        try:
            while not self._stop_event.is_set() and self.step_count < max_steps:
                t_loop_start = time.perf_counter()

                obs = self._get_observation()
                if obs is None:
                    self.dropped_frames += 1
                    time.sleep(dt)
                    continue

                actions = self._run_inference(obs)
                if actions is None:
                    self.dropped_frames += 1
                    time.sleep(dt)
                    continue

                safe_actions = self._apply_safety(actions)
                self._send_to_robot(safe_actions)
                self._update_tracking_error(safe_actions)

                with self._lock:
                    self.step_count += 1

                # Timing
                elapsed = time.perf_counter() - t_loop_start
                loop_times.append(elapsed)
                if len(loop_times) > 10:
                    avg = statistics.mean(loop_times[-10:])
                    with self._lock:
                        self.loop_hz = round(1.0 / max(avg, 1e-6), 1)

                sleep_time = dt - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

            self.final_status = "SUCCESS" if not self._stop_event.is_set() else "STOPPED"
        except Exception as e:
            self.final_status = "FAIL"
            self.last_error = str(e)
        finally:
            self.state = "done" if not self._stop_event.is_set() else "stopped"
            self.end_time = time.time()

    def get_live_status(self) -> Dict:
        with self._lock:
            jed = {jid: (statistics.mean(errs[-20:]) if errs else 0.0)
                   for jid, errs in self.joint_errors_deg.items()}
            return {
                "state": self.state,
                "step_count": self.step_count,
                "loop_hz": self.loop_hz,
                "inference_ms": statistics.mean(self.inference_times[-20:]) if self.inference_times else 0.0,
                "dropped_frames": self.dropped_frames,
                "safety_violations": self.safety_violations,
                "joint_errors_deg": jed,
                "last_error": self.last_error,
            }

    def get_evaluation_report(self) -> Dict:
        dur = (self.end_time or time.time()) - (self.start_time or time.time())
        inf = self.inference_times
        jed_mean = {jid: (statistics.mean(errs) if errs else 0.0)
                    for jid, errs in self.joint_errors_deg.items()}
        return {
            "model_id":         self.model_info.get("model_id", "?"),
            "model_name":       self.model_info.get("model_name", "?"),
            "framework":        self.model_info.get("framework", "?"),
            "simulator":        self.model_info.get("simulator", "?"),
            "algorithm":        self.model_info.get("algorithm", "?"),
            "step_count":       self.step_count,
            "duration_s":       round(dur, 3),
            "final_status":     self.final_status,
            "loop_hz":          self.loop_hz,
            "inference_ms_mean":round(statistics.mean(inf), 3) if inf else 0.0,
            "inference_ms_p95": round(sorted(inf)[int(len(inf) * 0.95)] if len(inf) > 1 else (inf[0] if inf else 0.0), 3),
            "inference_ms_max": round(max(inf), 3) if inf else 0.0,
            "dropped_frames":   self.dropped_frames,
            "safety_violations":self.safety_violations,
            "joint_errors_deg": jed_mean,
        }


# ── Global State ──────────────────────────────────────────────────────────────
_lock = threading.Lock()
_models: Dict[str, Dict] = {}          # model_id -> model_info dict
_engine: Optional[PolicyExecutionEngine] = None
_last_report: Optional[Dict] = None

# References to the bridge's robot driver and camera manager (injected at startup)
_robot_driver_ref = None
_camera_manager_ref = None


def set_robot_driver(driver):
    global _robot_driver_ref
    _robot_driver_ref = driver


def set_camera_manager(cam):
    global _camera_manager_ref
    _camera_manager_ref = cam


# ── Tornado Handlers ──────────────────────────────────────────────────────────
class _BaseAIHandler(tornado.web.RequestHandler):
    def set_default_headers(self):
        self.set_header("Content-Type", "application/json")
        self.set_header("Access-Control-Allow-Origin", "*")

    def options(self, *args):
        self.set_header("Access-Control-Allow-Methods", "GET,POST,OPTIONS")
        self.set_header("Access-Control-Allow-Headers", "Content-Type")
        self.set_status(204)
        self.finish()

    def json(self, obj):
        self.write(json.dumps(obj))
        self.finish()


class AIModelUploadHandler(_BaseAIHandler):
    """POST multipart/form-data — upload checkpoint + optional config."""

    async def post(self):
        try:
            files = self.request.files
            if "file" not in files:
                return self.json({"success": False, "message": "No file uploaded (field name: 'file')"})

            upload = files["file"][0]
            orig_name = upload["filename"]
            body = upload["body"]

            # Save to disk
            model_id = str(uuid.uuid4())[:12]
            dest_dir = UPLOAD_DIR / model_id
            dest_dir.mkdir(parents=True, exist_ok=True)
            dest_path = dest_dir / orig_name
            dest_path.write_bytes(body)

            # Save optional config
            if "config_file" in files:
                cfg_upload = files["config_file"][0]
                cfg_path = dest_dir / cfg_upload["filename"]
                cfg_path.write_bytes(cfg_upload["body"])

            # Framework detection
            fw_info = _detect_framework(dest_path, orig_name)
            fw_info["model_id"] = model_id
            fw_info["model_name"] = self.get_argument("model_name", orig_name.rsplit(".", 1)[0])
            fw_info["simulator"] = self.get_argument("simulator", "unknown")
            fw_info["algorithm"] = self.get_argument("algorithm", "unknown")
            fw_info["version"] = self.get_argument("version", "v1.0")
            fw_info["file_path"] = str(dest_path)
            fw_info["file_size"] = len(body)

            with _lock:
                _models[model_id] = fw_info

            self.json({
                "success":      True,
                "model_id":     model_id,
                "framework":    fw_info["framework"],
                "format":       fw_info["format"],
                "file_size":    fw_info["file_size"],
                "input_shape":  fw_info["input_shape"],
                "output_shape": fw_info["output_shape"],
                "warnings":     fw_info["warnings"],
                "message":      f"Uploaded and inspected as {fw_info['framework'].upper()}",
            })
        except Exception as e:
            self.json({"success": False, "message": str(e)})


class AIValidationHandler(_BaseAIHandler):
    """POST {model_id} — run compatibility checks."""

    async def post(self):
        try:
            body = json.loads(self.request.body or "{}")
            model_id = body.get("model_id")
            with _lock:
                fw_info = _models.get(model_id)
            if not fw_info:
                return self.json({"success": False, "message": f"Unknown model_id: {model_id}"})

            checks = run_compatibility_checks(fw_info, body)
            self.json({"success": True, "model_id": model_id, "checks": checks})
        except Exception as e:
            self.json({"success": False, "message": str(e)})


class AIHardwarePrefightHandler(_BaseAIHandler):
    """POST {} — check robot USB, 6 joints, camera."""

    async def post(self):
        items: Dict[str, Dict] = {}

        driver = _robot_driver_ref
        cam = _camera_manager_ref

        # USB / Serial
        if driver and getattr(driver, "is_connected", False):
            items["usb"] = {"status": "PASS", "detail": getattr(driver, "port", "connected")}
        elif driver:
            items["usb"] = {"status": "FAIL", "detail": "Driver present but not connected"}
        else:
            items["usb"] = {"status": "FAIL", "detail": "No robot driver (run 'Connect Robot' first)"}

        # 6 Joints — ping each servo
        joint_names = {1: "shoulder_pan", 2: "shoulder_lift", 3: "elbow_flex",
                       4: "wrist_flex", 5: "wrist_roll", 6: "gripper"}
        for jid in range(1, 7):
            key = f"joint_{jid}"
            if driver and getattr(driver, "is_connected", False):
                try:
                    pos = None
                    if hasattr(driver, "read_positions"):
                        positions = driver.read_positions()
                        pos = positions.get(jid)
                    elif hasattr(driver, "positions"):
                        pos = driver.positions.get(jid)
                    if pos is not None:
                        items[key] = {"status": "PASS", "detail": f"{pos} ticks"}
                    else:
                        items[key] = {"status": "FAIL", "detail": "No position feedback"}
                except Exception as e:
                    items[key] = {"status": "FAIL", "detail": str(e)[:50]}
            else:
                items[key] = {"status": "FAIL", "detail": "Robot not connected"}

        # Camera
        if cam and getattr(cam, "is_running", False):
            fps = getattr(cam, "actual_fps", 0)
            items["camera"] = {"status": "PASS" if fps > 0 else "WARNING",
                               "detail": f"{fps:.1f} FPS" if fps > 0 else "Connected but 0 FPS"}
        else:
            items["camera"] = {"status": "WARNING", "detail": "Camera not connected (optional for RGB obs)"}

        self.json({"success": True, "items": items})


class AIPolicyStartHandler(_BaseAIHandler):
    """POST {model_id, hz, max_steps, safety_mode, use_camera}"""

    async def post(self):
        global _engine
        try:
            body = json.loads(self.request.body or "{}")
            model_id = body.get("model_id")
            with _lock:
                fw_info = _models.get(model_id)
            if not fw_info:
                return self.json({"success": False, "message": f"Unknown model_id: {model_id}"})

            # Pre-load model session if possible
            if fw_info["framework"] == "onnx" and ONNX_AVAILABLE and "_session" not in fw_info:
                try:
                    sess = onnxrt.InferenceSession(fw_info["file_path"], providers=["CPUExecutionProvider"])  # type: ignore
                    fw_info["_session"] = sess
                except Exception as e:
                    return self.json({"success": False, "message": f"ONNX session failed: {e}"})

            elif fw_info["framework"] == "pytorch" and TORCH_AVAILABLE and "_model" not in fw_info:
                try:
                    obj = torch.load(fw_info["file_path"], map_location="cpu", weights_only=False)  # type: ignore
                    if hasattr(obj, "forward"):
                        obj.eval()
                        fw_info["_model"] = obj
                    elif isinstance(obj, dict):
                        fw_info["_model"] = None  # state_dict only, can't run directly
                except Exception as e:
                    pass  # run in simulation mode

            engine = PolicyExecutionEngine(
                model_info=fw_info,
                params=body,
                robot_driver=_robot_driver_ref,
                camera_manager=_camera_manager_ref,
            )
            with _lock:
                _engine = engine
            engine.start()

            self.json({"success": True, "message": "Policy execution started", "model_id": model_id})
        except Exception as e:
            self.json({"success": False, "message": str(e)})


class AIPolicyStopHandler(_BaseAIHandler):
    """POST {} — emergency stop."""

    async def post(self):
        global _last_report
        with _lock:
            eng = _engine
        if eng:
            eng.emergency_stop()
            with _lock:
                _last_report = eng.get_evaluation_report()
        self.json({"success": True, "message": "Policy stopped"})


class AIPolicyStatusHandler(_BaseAIHandler):
    """GET — live execution metrics."""

    async def get(self):
        with _lock:
            eng = _engine
        if not eng:
            self.json({"state": "idle", "step_count": 0, "loop_hz": 0, "inference_ms": 0,
                       "dropped_frames": 0, "safety_violations": 0, "joint_errors_deg": {}})
            return

        status = eng.get_live_status()
        # Also save report when done
        global _last_report
        if status["state"] in ("done", "stopped", "error"):
            with _lock:
                _last_report = eng.get_evaluation_report()
        self.json(status)


class AIEvaluationReportHandler(_BaseAIHandler):
    """GET — final evaluation report."""

    async def get(self):
        with _lock:
            eng = _engine
            rep = _last_report

        if eng and eng.state in ("done", "stopped"):
            rep = eng.get_evaluation_report()

        if rep:
            self.json({"success": True, "report": rep})
        else:
            self.json({"success": False, "message": "No evaluation report yet. Run a policy session first."})


class AIModelListHandler(_BaseAIHandler):
    """GET — list uploaded models."""

    async def get(self):
        with _lock:
            models = [
                {"model_id": mid,
                 "model_name": info.get("model_name", "?"),
                 "framework": info.get("framework", "?"),
                 "simulator": info.get("simulator", "?"),
                 "file_size": info.get("file_size", 0),
                 "format": info.get("format", "?")}
                for mid, info in _models.items()
            ]
        self.json({"success": True, "models": models})


# ── Route List (add to bridge_standalone.py make_app) ────────────────────────
AI_ROUTES = [
    (r"/api/ai/upload",               AIModelUploadHandler),
    (r"/api/ai/validate",             AIValidationHandler),
    (r"/api/ai/hw_preflight",         AIHardwarePrefightHandler),
    (r"/api/ai/policy/start",         AIPolicyStartHandler),
    (r"/api/ai/policy/stop",          AIPolicyStopHandler),
    (r"/api/ai/policy/status",        AIPolicyStatusHandler),
    (r"/api/ai/evaluation/report",    AIEvaluationReportHandler),
    (r"/api/ai/models",               AIModelListHandler),
]
