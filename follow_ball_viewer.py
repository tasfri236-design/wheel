"""Tennis ball tracker that reuses camera_stream.py's proven Picamera2 pipeline.

This is a drop-in alternative to follow_ball.py for Raspberry Pi setups where
camera_viewer.py / camera_stream.py works but follow_ball.py's own CSI shim
fails to grab the camera. Instead of opening Picamera2 ourselves, we start
camera_stream's capture loop (same config as camera_viewer.py) and read frames
directly off the running Picamera2 instance.

Bonus: an MJPEG preview is exposed (default port 8001 to avoid colliding with
FastAPI on 8000) so you can keep an eye on what the tracker sees from a browser.

Usage (on the Pi):
    # 1. Start the FastAPI backend in another terminal:
    python -m uvicorn app.main:app --host 0.0.0.0 --port 8000

    # 2. Run the tracker (this script):
    python follow_ball_viewer.py --show
    python follow_ball_viewer.py --calibrate
    python follow_ball_viewer.py --dry-run
    python follow_ball_viewer.py --stream-port 8001  # change MJPEG port
    python follow_ball_viewer.py --no-stream          # capture only, no HTTP server

    # While running, open http://<pi-ip>:8001/ for a live preview.
"""
from __future__ import annotations

import argparse
import signal
import sys
import time
from typing import Optional

import cv2
import imutils
import numpy as np
import requests

import camera_stream
from camera_stream import start_camera_stream, stop_camera_stream
from follow_ball import (
    BallTracker,
    CommandClient,
    MAX_ATTAINABLE_RPM,
    PID,
    YELLOW_LOWER_DEFAULT,
    YELLOW_UPPER_DEFAULT,
    make_calibration_window,
    parse_hsv,
    read_calibration,
)


def _grab_frame() -> Optional[np.ndarray]:
    """Pull the latest frame from camera_stream's running Picamera2 instance.

    camera_stream._capture_loop is already running in a daemon thread; we just
    read a fresh array so we don't pay JPEG round-trip cost. Picamera2's
    capture_array() is safe to call from multiple threads. Falls back to
    decoding the cached JPEG if the picam handle isn't ready yet.
    """
    picam = camera_stream._picam2
    if picam is not None:
        try:
            frame = picam.capture_array()
            return cv2.rotate(frame, cv2.ROTATE_180)
        except Exception:
            pass

    with camera_stream._frame_lock:
        jpeg = camera_stream._latest_jpeg
    if jpeg is None:
        return None
    arr = np.frombuffer(jpeg, dtype=np.uint8)
    decoded = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    return decoded


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="Tennis ball tracker -> reaction wheel PID (camera_stream backend)",
    )
    p.add_argument("--api", default="http://127.0.0.1:8000",
                   help="Base URL of the FastAPI backend")
    p.add_argument("--width", type=int, default=400,
                   help="Resize width before tracking (lower = faster)")
    p.add_argument("--dead-zone", type=float, default=0.05,
                   help="Fraction of frame width treated as 'centered' (brake zone)")
    p.add_argument("--stop-radius", type=float, default=80.0,
                   help="If the ball's enclosing radius >= this, brake.")
    p.add_argument("--kp", type=float, default=0.08)
    p.add_argument("--ki", type=float, default=0.0)
    p.add_argument("--kd", type=float, default=0.02)
    p.add_argument("--max-rpm", type=float, default=float(MAX_ATTAINABLE_RPM),
                   help="Upper bound on |target_rpm| sent to /api/command")
    p.add_argument("--invert", action="store_true",
                   help="Swap cw/ccw mapping")
    p.add_argument("--hsv-lower", type=parse_hsv, default=YELLOW_LOWER_DEFAULT,
                   help="HSV lower bound 'H,S,V'")
    p.add_argument("--hsv-upper", type=parse_hsv, default=YELLOW_UPPER_DEFAULT,
                   help="HSV upper bound 'H,S,V'")
    p.add_argument("--show", action="store_true",
                   help="Display preview windows (requires X / cv2.imshow)")
    p.add_argument("--calibrate", action="store_true",
                   help="Live HSV trackbars to dial in --hsv-lower/--hsv-upper")
    p.add_argument("--dry-run", action="store_true",
                   help="Log commands instead of POSTing to FastAPI")
    p.add_argument("--stream-port", type=int, default=8001,
                   help="HTTP port for the MJPEG preview (default 8001 to avoid FastAPI on 8000)")
    p.add_argument("--no-stream", action="store_true",
                   help="Skip starting the HTTP MJPEG server (capture loop still runs)")
    p.add_argument("--cam-width", type=int, default=640)
    p.add_argument("--cam-height", type=int, default=480)
    p.add_argument("--fps", type=int, default=30)
    return p


def main(argv: Optional[list[str]] = None) -> int:
    args = build_parser().parse_args(argv)

    cv2.setUseOptimized(True)
    try:
        cv2.setNumThreads(2)
    except Exception:
        pass

    if not args.dry_run:
        try:
            health = requests.get(f"{args.api.rstrip('/')}/api/health", timeout=1.0)
            health.raise_for_status()
            print(f">> API reachable at {args.api} -> {health.json()}")
        except requests.RequestException as exc:
            print(
                f">> ERROR: cannot reach FastAPI at {args.api}: {exc}\n"
                f"   Start it first (on the same host):\n"
                f"   python -m uvicorn app.main:app --host 0.0.0.0 --port 8000\n"
                f"   Or re-run with --dry-run to skip the API."
            )
            return 2

    if args.no_stream:
        # Cheap trick: start_camera_stream listens on host="127.0.0.1" port we pick;
        # bind to a port we know nothing else uses and just don't visit it.
        print(">> starting camera capture (no HTTP preview)")
        start_camera_stream(
            resolution=(args.cam_width, args.cam_height),
            fps=args.fps,
            host="127.0.0.1",
            port=args.stream_port,
        )
    else:
        print(f">> starting camera capture + MJPEG preview on :{args.stream_port}")
        start_camera_stream(
            resolution=(args.cam_width, args.cam_height),
            fps=args.fps,
            port=args.stream_port,
        )

    # Wait for the capture thread to actually publish a frame.
    print(">> waiting for first frame...")
    deadline = time.time() + 8.0
    probe = None
    while time.time() < deadline:
        probe = _grab_frame()
        if probe is not None:
            break
        time.sleep(0.1)
    if probe is None:
        print(
            ">> ERROR: camera_stream started but never produced a frame.\n"
            "   - Check `ps -ef | grep camera` for another process holding it.\n"
            "   - Try: python camera_viewer.py and confirm the browser preview works.\n"
            "   - If camera_viewer.py also fails, this is a libcamera/firmware issue."
        )
        stop_camera_stream()
        return 4
    print(f">> camera ready: {probe.shape[1]}x{probe.shape[0]}")

    if args.calibrate:
        make_calibration_window()

    tracker = BallTracker(args.hsv_lower, args.hsv_upper)
    pid = PID(kp=args.kp, ki=args.ki, kd=args.kd)
    client = CommandClient(args.api, dry_run=args.dry_run)

    stop_hyst_px = args.stop_radius * 0.85
    stopped_by_radius = False
    last_t = time.monotonic()

    interrupted = {"flag": False}

    def _sigint(_signum, _frame):
        interrupted["flag"] = True

    signal.signal(signal.SIGINT, _sigint)

    print(">> tracking. Ctrl+C to quit.")
    try:
        while not interrupted["flag"]:
            raw = _grab_frame()
            if raw is None:
                time.sleep(0.01)
                continue

            frame = imutils.resize(raw, width=args.width)
            h, w = frame.shape[:2]
            center_x = w // 2
            dead_zone_px = int(round(args.dead_zone * w))

            if args.calibrate:
                lower, upper = read_calibration()
                tracker.set_range(lower, upper)

            result, mask = tracker.detect(frame)

            now = time.monotonic()
            dt = max(1e-3, now - last_t)
            last_t = now

            if result is None:
                client.brake(now)
                pid.reset()
            else:
                cx, cy, radius = result
                error = cx - center_x

                if radius >= args.stop_radius:
                    stopped_by_radius = True
                elif stopped_by_radius and radius < stop_hyst_px:
                    stopped_by_radius = False

                if stopped_by_radius or abs(error) <= dead_zone_px:
                    client.brake(now)
                    pid.reset()
                else:
                    u = pid.step(error, dt)
                    positive_right = (u > 0)
                    if args.invert:
                        positive_right = not positive_right
                    direction = "cw" if positive_right else "ccw"
                    target_rpm = min(float(args.max_rpm), abs(u))
                    client.spin(target_rpm, direction, now)

            if args.show or args.calibrate:
                vis = frame.copy()
                if result is not None:
                    cx, cy, radius = result
                    cv2.circle(vis, (cx, cy), int(radius), (0, 255, 255), 2)
                    cv2.circle(vis, (cx, cy), 3, (0, 0, 255), -1)
                cv2.line(vis, (center_x, 0), (center_x, h), (255, 255, 255), 1)
                left = center_x - dead_zone_px
                right = center_x + dead_zone_px
                cv2.line(vis, (left, 0), (left, h), (0, 200, 0), 1)
                cv2.line(vis, (right, 0), (right, h), (0, 200, 0), 1)
                cv2.imshow("follow_ball_viewer", vis)
                if args.calibrate:
                    cv2.imshow("mask", mask)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
    finally:
        try:
            client.brake(time.monotonic())
        except Exception:
            pass
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        try:
            stop_camera_stream()
        except Exception:
            pass
        print(">> stopped.")

    return 0


if __name__ == "__main__":
    sys.exit(main())
