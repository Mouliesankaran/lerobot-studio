#!/usr/bin/env python3
"""
SO-101 camera acceptance test - run it ON EACH TARGET MACHINE while the bridge is running.

    python tests/camera_acceptance.py                       # auto: first real camera found
    python tests/camera_acceptance.py --type realsense      # force a camera type
    python tests/camera_acceptance.py --type synthetic      # pipeline-only (bridge started with --test-camera)
    python tests/camera_acceptance.py --url http://192.168.1.20:8080

Exit code 0 = all criteria passed.  Prints a PASS/FAIL table you can paste in the test log.
Only needs the Python standard library (+ numpy/opencv only if installed, for pixel checks).
"""
import argparse, json, sys, time, urllib.request

def call(base, path, body=None, timeout=20):
    req = urllib.request.Request(base + path, json.dumps(body).encode() if body is not None else None,
                                 {"Content-Type": "application/json"} if body is not None else {})
    return urllib.request.urlopen(req, timeout=timeout).read()

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default="http://localhost:8080")
    ap.add_argument("--type", default="auto", choices=["auto", "usb", "realsense", "orbbec", "synthetic"])
    ap.add_argument("--min-fps", type=float, default=10.0)
    a = ap.parse_args()
    B = a.url.rstrip("/")
    results = []
    def check(name, ok, detail=""):
        results.append((name, bool(ok), detail)); print(f"[{'PASS' if ok else 'FAIL'}] {name} {detail}")

    try:
        info = json.loads(call(B, "/api/system/info"))
        check("T01 bridge reachable + serves UI", info.get("ok"), f"web_dir={info.get('web_dir')}")
        cam = info.get("camera", {})
        check("T02 OpenCV usable", cam.get("opencv"), f"opencv={cam.get('opencv')} err={cam.get('opencv_error')}")
        print("      platform:", cam.get("platform"), "python:", cam.get("python"),
              "realsense:", cam.get("pyrealsense2"), "orbbec:", cam.get("pyorbbecsdk"))
        index_ok = len(call(B, "/")) > 10000
        check("T03 index.html served by bridge (not 404)", index_ok)

        devs = json.loads(call(B, "/api/camera/devices?refresh=1"))["devices"]
        order = [a.type] if a.type != "auto" else ["realsense", "orbbec", "usb", "synthetic"]
        dev = next((d for t in order for d in devs if d["type"] == t), None)
        check("T04 device detected", dev is not None, f"{dev['name'] if dev else 'none: ' + str([d['type'] for d in devs])}")
        if not dev:
            return finish(results)

        r = json.loads(call(B, "/api/camera/connect", {"device_id": dev["id"], "camera_type": dev["type"],
                                                         "width": 640, "height": 480, "fps": 30}, timeout=60))
        check("T05 connect", r.get("success"), r.get("message", ""))
        if not r.get("success"):
            return finish(results)
        time.sleep(2.0)

        try:
            import numpy as np, cv2
        except Exception:
            np = cv2 = None
        shapes = {}
        for mode in ("rgb", "depth", "side_by_side"):
            data = call(B, f"/api/camera/snapshot?mode={mode}")
            ok = data[:2] == b"\xff\xd8"
            if cv2 is not None and ok:
                img = cv2.imdecode(np.frombuffer(data, np.uint8), 1)
                shapes[mode] = img.shape
                if mode == "rgb":
                    ok = img.std() > 3          # not a black / blank frame
            check(f"T06 snapshot {mode} is a valid JPEG", ok, str(shapes.get(mode, "")))
        if cv2 is not None and len(shapes) == 3:
            check("T07 RGB+Depth width == 2x RGB width", shapes["side_by_side"][1] in (shapes["rgb"][1] * 2, 1280),
                  f"{shapes['side_by_side']} vs {shapes['rgb']}")

        st = json.loads(call(B, "/api/camera/status"))
        check(f"T08 live FPS >= {a.min_fps}", st["actual_fps"] >= a.min_fps, f"{st['actual_fps']} fps, dropped={st['dropped_frames']}")
        want_real = dev["type"] in ("realsense", "orbbec", "synthetic")
        check("T09 depth flag matches hardware", st["depth_is_real"] == want_real,
              f"depth_is_real={st['depth_is_real']} expected={want_real}")

        t0, frames, buf = time.time(), 0, b""
        resp = urllib.request.urlopen(f"{B}/api/camera/stream?mode=side_by_side", timeout=15)
        while time.time() - t0 < 3:
            buf += resp.read(8192)
        frames = buf.count(b"--frame"); resp.close()
        check("T10 MJPEG stream delivers frames", frames >= 15, f"{frames} frames in 3 s")

        call(B, "/api/camera/disconnect", {})
        time.sleep(0.5)
        st = json.loads(call(B, "/api/camera/status"))
        check("T11 clean disconnect", not st["connected"])
        r = json.loads(call(B, "/api/camera/connect", {"device_id": dev["id"], "camera_type": dev["type"]}, timeout=60))
        check("T12 reconnect after disconnect", r.get("success"), r.get("message", ""))
        call(B, "/api/camera/disconnect", {})
    except Exception as e:
        check("EXC unexpected error", False, repr(e))
    return finish(results)

def finish(results):
    bad = [r for r in results if not r[1]]
    print(f"\n{len(results) - len(bad)}/{len(results)} passed")
    return 1 if bad else 0

if __name__ == "__main__":
    sys.exit(main())
