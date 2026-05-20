#!/usr/bin/env python3
"""
HX1A Pi Zero 2W — Arducam ToF Camera Test
Hosts a web server that streams color-mapped depth images over HTTP.

Usage:
    python3 app.py [--mock] [--host 0.0.0.0] [--port 5000]

Without --mock: attempts to read from VL53L5CX ToF sensor via I2C
With --mock:    generates synthetic depth data for testing without hardware
"""

import argparse
import io
import logging
import time
import threading
from pathlib import Path

import cv2
import numpy as np
from flask import Flask, Response, jsonify, render_template_string

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
)
log = logging.getLogger(__name__)

# ───────────────────────────────────────────────────────────────────
#  TOF SENSOR ABSTRACTION
# ───────────────────────────────────────────────────────────────────

class ToFCamera:
    """Base interface — override _capture_raw() for your sensor."""

    WIDTH = 64
    HEIGHT = 48
    MIN_MM = 20
    MAX_MM = 4000

    def capture_depth(self) -> np.ndarray | None:
        """Returns depth frame in mm (HxW uint16) or None on failure."""
        raise NotImplementedError


class VL53L5CXCamera(ToFCamera):
    """Real Arducam VL53L5CX ToF camera via I2C."""

    def __init__(self):
        try:
            from vl53l5cx import VL53L5CX
            self.dev = VL53L5CX()
            self.dev.start_ranging()
            self.running = True
            log.info("VL53L5CX ToF camera initialized")
        except Exception as e:
            log.error("Failed to init VL53L5CX: %s", e)
            raise

    def capture_depth(self) -> np.ndarray | None:
        try:
            data = self.dev.get_data()
            dist = data.distance_mm  # 1D array, 64*48 = 3072
            frame = np.array(dist, dtype=np.uint16).reshape(self.HEIGHT, self.WIDTH)
            return frame
        except Exception as e:
            log.warning("Capture failed: %s", e)
            return None

    def close(self):
        if self.running:
            try:
                self.dev.stop_ranging()
            except Exception:
                pass
            self.running = False


class MockCamera(ToFCamera):
    """Synthetic depth data — sine waves + noise for testing."""

    def __init__(self):
        self.t = 0.0
        log.info("Mock ToF camera initialized (no hardware)")

    def capture_depth(self) -> np.ndarray:
        self.t += 0.05
        y, x = np.mgrid[0:self.HEIGHT, 0:self.WIDTH]
        depth = (
            500
            + 200 * np.sin(x / 10 + self.t)
            + 150 * np.cos(y / 8 - self.t * 0.7)
            + np.random.randint(-30, 30, (self.HEIGHT, self.WIDTH), dtype=np.int16)
        )
        depth = np.clip(depth, self.MIN_MM, self.MAX_MM).astype(np.uint16)
        return depth

    def close(self):
        pass


# ───────────────────────────────────────────────────────────────────
#  DEPTH → COLOR MAPPING
# ───────────────────────────────────────────────────────────────────

def depth_to_color(depth_mm: np.ndarray, min_mm: int = 20, max_mm: int = 4000) -> bytes:
    """Convert depth frame (mm) to color-mapped JPEG bytes."""
    norm = np.clip((depth_mm.astype(np.float32) - min_mm) / (max_mm - min_mm), 0, 1)
    colored = cv2.applyColorMap((norm * 255).astype(np.uint8), cv2.COLORMAP_TURBO)

    h, w = colored.shape[:2]
    scale = 6
    big = cv2.resize(colored, (w * scale, h * scale), interpolation=cv2.INTER_NEAREST)

    # Overlay stats
    valid = depth_mm[depth_mm > min_mm]
    if len(valid) > 0:
        avg = valid.mean()
        cv2.putText(big, f"AVG: {avg:.0f}mm", (10, 30),
                     cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(big, f"MIN: {valid.min():.0f}mm", (10, 60),
                     cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(big, f"MAX: {valid.max():.0f}mm", (10, 90),
                     cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

    _, jpeg = cv2.imencode(".jpg", big, [cv2.IMWRITE_JPEG_QUALITY, 80])
    return jpeg.tobytes()


# ───────────────────────────────────────────────────────────────────
#  FLASK APP
# ───────────────────────────────────────────────────────────────────

app = Flask(__name__)
camera: ToFCamera | None = None
latest_frame: np.ndarray | None = None
frame_lock = threading.Lock()
stats = {"fps": 0, "frames": 0, "started": time.time()}


def _capture_loop():
    """Background thread that continuously grabs depth frames."""
    global latest_frame
    fps_times = []

    while True:
        t0 = time.monotonic()
        try:
            depth = camera.capture_depth()
            if depth is not None:
                with frame_lock:
                    latest_frame = depth.copy()
                stats["frames"] += 1
        except Exception as e:
            log.error("Capture error: %s", e)

        elapsed = time.monotonic() - t0
        fps_times.append(elapsed)
        if len(fps_times) > 30:
            fps_times.pop(0)
        stats["fps"] = round(1.0 / (sum(fps_times) / len(fps_times)), 1)

        time.sleep(max(0, 0.033 - elapsed))  # ~30 FPS cap


@app.route("/")
def index():
    return render_template_string(HTML_TEMPLATE)


@app.route("/stream.mjpg")
def stream():
    def generate():
        while True:
            with frame_lock:
                if latest_frame is not None:
                    jpeg = depth_to_color(latest_frame)
                    yield (b"--frame\r\n"
                           b"Content-Type: image/jpeg\r\n\r\n" + jpeg + b"\r\n")
            time.sleep(0.033)

    return Response(generate(), mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/api/stats")
def api_stats():
    uptime = time.time() - stats["started"]
    return jsonify({
        "fps": stats["fps"],
        "total_frames": stats["frames"],
        "uptime_s": round(uptime, 1),
        "sensor": type(camera).__name__,
        "resolution": f"{camera.WIDTH}x{camera.HEIGHT}",
    })


HTML_TEMPLATE = """
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>HX1A ToF Depth Feed</title>
<style>
  * { margin: 0; padding: 0; box-sizing: border-box; }
  body {
    background: #0a0a0a; color: #e0e0e0;
    font-family: 'Segoe UI', system-ui, sans-serif;
    display: flex; flex-direction: column; align-items: center;
    min-height: 100vh; padding: 20px;
  }
  h1 { font-size: 1.4rem; margin-bottom: 12px; color: #00e5ff; }
  .feed {
    border: 2px solid #1a1a2e; border-radius: 8px;
    max-width: 95vw; image-rendering: pixelated;
    box-shadow: 0 0 20px rgba(0,229,255,0.15);
  }
  #stats {
    margin-top: 14px; font-size: 0.85rem;
    color: #888; font-family: monospace;
  }
</style>
</head>
<body>
  <h1>HX1A — Arducam ToF Depth Feed</h1>
  <img class="feed" src="/stream.mjpg" alt="Depth stream">
  <div id="stats">Loading...</div>
<script>
  setInterval(async () => {
    try {
      const s = await (await fetch('/api/stats')).json();
      document.getElementById('stats').textContent =
        `${s.sensor} | ${s.resolution} | ${s.fps} FPS | ${s.total_frames} frames | ${s.uptime_s}s uptime`;
    } catch {}
  }, 1000);
</script>
</body>
</html>
"""


# ───────────────────────────────────────────────────────────────────
#  MAIN
# ───────────────────────────────────────────────────────────────────

def main():
    global camera

    parser = argparse.ArgumentParser(description="HX1A ToF Camera Test")
    parser.add_argument("--mock", action="store_true", help="Use synthetic depth data")
    parser.add_argument("--host", default="0.0.0.0", help="Bind address")
    parser.add_argument("--port", type=int, default=5000, help="Port")
    args = parser.parse_args()

    if args.mock:
        camera = MockCamera()
    else:
        try:
            camera = VL53L5CXCamera()
        except Exception:
            log.warning("Hardware init failed, falling back to mock mode")
            camera = MockCamera()

    t = threading.Thread(target=_capture_loop, daemon=True)
    t.start()

    log.info("Web server → http://%s:%d", args.host, args.port)
    app.run(host=args.host, port=args.port, threaded=True)


if __name__ == "__main__":
    main()
