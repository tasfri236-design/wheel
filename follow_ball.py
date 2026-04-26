"""Tennis ball tracker that steers the reaction wheel via the FastAPI /api/command endpoint.

Pipeline: OpenCV resize -> GaussianBlur -> HSV inRange -> erode/dilate ->
largest contour -> minEnclosingCircle -> PID on pixel error -> motor command.

Usage:
    python follow_ball.py --api http://127.0.0.1:8000
    python follow_ball.py --show                 # preview window
    python follow_ball.py --calibrate            # HSV trackbars
    python follow_ball.py --dry-run              # log only, no HTTP
    python follow_ball.py --invert               # flip cw/ccw
    python follow_ball.py --csi                  # Picamera2 instead of USB
"""
from __future__ import annotations

import argparse
import signal
import sys
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import imutils
import numpy as np
import requests
from imutils.video import VideoStream


YELLOW_LOWER_DEFAULT = (25, 50, 50)
YELLOW_UPPER_DEFAULT = (45, 255, 255)

MIN_RADIUS_PX = 8
MAX_ATTAINABLE_RPM = 60
COMMAND_MIN_INTERVAL = 1.0 / 20.0
COMMAND_RPM_EPSILON = 2.0


def parse_hsv(s: str) -> Tuple[int, int, int]:
    try:
        parts = [int(p.strip()) for p in s.split(",")]
        if len(parts) != 3:
            raise ValueError
        return (parts[0], parts[1], parts[2])
    except Exception:
        raise argparse.ArgumentTypeError(f"HSV must be 'H,S,V' integers, got: {s!r}")


@dataclass
class PID:
    kp: float
    ki: float
    kd: float
    integral_clamp: float = 50.0
    _integral: float = 0.0
    _prev_error: Optional[float] = None

    def reset(self) -> None:
        self._integral = 0.0
        self._prev_error = None

    def step(self, error: float, dt: float) -> float:
        if dt <= 0:
            return self.kp * error
        self._integral += error * dt
        if self._integral > self.integral_clamp:
            self._integral = self.integral_clamp
        elif self._integral < -self.integral_clamp:
            self._integral = -self.integral_clamp
        derivative = 0.0 if self._prev_error is None else (error - self._prev_error) / dt
        self._prev_error = error
        return self.kp * error + self.ki * self._integral + self.kd * derivative


class CsiStream:
    """Picamera2 shim exposing VideoStream-like read()/stop().

    Mirrors the working config in camera_stream.py:
    - RGB888 format (numpy array is already in BGR byte order - do NOT cvtColor).
    - Explicit FrameRate in controls.
    - Optional 180 deg rotation for upside-down-mounted modules.
    """

    def __init__(self, width: int = 640, height: int = 480, fps: int = 30, rotate: int = 180) -> None:
        from picamera2 import Picamera2

        # Avoid Picamera2's cryptic "list index out of range" when no camera is enumerated.
        try:
            camera_info = Picamera2.global_camera_info()
        except Exception as exc:
            raise RuntimeError(f"failed to enumerate CSI cameras: {exc}") from exc
        if not camera_info:
            raise RuntimeError(
                "no CSI cameras found (libcamera reports none). "
                "Check camera cable/orientation and enable camera support in raspi-config."
            )

        try:
            self._picam = Picamera2()
            cfg = self._picam.create_video_configuration(
                main={"size": (width, height), "format": "RGB888"},
                controls={"FrameRate": fps},
            )
            self._picam.configure(cfg)
            self._picam.start()
        except Exception as exc:
            msg = str(exc)
            if "list index out of range" in msg:
                raise RuntimeError(
                    "CSI camera initialization failed: no usable camera device was selected. "
                    "Run `rpicam-hello --list-cameras` to confirm detection."
                ) from exc
            raise RuntimeError(f"CSI camera initialization failed: {msg}") from exc

        self._rotate = rotate % 360

    def read(self) -> Optional[np.ndarray]:
        arr = self._picam.capture_array()
        if arr is None:
            return None
        # Picamera2's "RGB888" is stored as BGR in the numpy buffer -> OpenCV-ready.
        if self._rotate == 90:
            return cv2.rotate(arr, cv2.ROTATE_90_CLOCKWISE)
        if self._rotate == 180:
            return cv2.rotate(arr, cv2.ROTATE_180)
        if self._rotate == 270:
            return cv2.rotate(arr, cv2.ROTATE_90_COUNTERCLOCKWISE)
        return arr

    def stop(self) -> None:
        try:
            self._picam.stop()
        except Exception:
            pass


class CommandClient:
    def __init__(self, api: str, dry_run: bool = False) -> None:
        self.api = api.rstrip("/")
        self.dry_run = dry_run
        self._session = requests.Session()
        self._last_post_t = 0.0
        self._last_rpm: Optional[float] = None
        self._last_dir: Optional[str] = None
        self._last_action: Optional[str] = None

    def brake(self, now: float) -> None:
        if self._last_action == "brake" and (now - self._last_post_t) < 0.5:
            return
        self._post({"action": "brake"}, now)
        self._last_action = "brake"
        self._last_rpm = None
        self._last_dir = None

    def spin(self, target_rpm: float, direction: str, now: float) -> None:
        direction_changed = direction != self._last_dir
        rpm_changed = (
            self._last_rpm is None or abs(target_rpm - self._last_rpm) >= COMMAND_RPM_EPSILON
        )
        throttled = (now - self._last_post_t) < COMMAND_MIN_INTERVAL
        if throttled and not direction_changed:
            return
        if not direction_changed and not rpm_changed and self._last_action == "spin":
            return
        self._post({"target_rpm": round(float(target_rpm), 1), "direction": direction}, now)
        self._last_action = "spin"
        self._last_rpm = float(target_rpm)
        self._last_dir = direction

    def _post(self, body: dict, now: float) -> None:
        self._last_post_t = now
        if self.dry_run:
            print(f"[dry-run] POST /api/command {body}")
            return
        try:
            self._session.post(f"{self.api}/api/command", json=body, timeout=0.5)
        except requests.RequestException as exc:
            print(f"[warn] /api/command failed: {exc}")


class BallTracker:
    def __init__(self, hsv_lower: Tuple[int, int, int], hsv_upper: Tuple[int, int, int]) -> None:
        self.hsv_lower = np.array(hsv_lower, dtype=np.uint8)
        self.hsv_upper = np.array(hsv_upper, dtype=np.uint8)
        self._hsv_buf: Optional[np.ndarray] = None

    def set_range(self, lower: Tuple[int, int, int], upper: Tuple[int, int, int]) -> None:
        self.hsv_lower = np.array(lower, dtype=np.uint8)
        self.hsv_upper = np.array(upper, dtype=np.uint8)

    def detect(self, frame_bgr: np.ndarray) -> Tuple[Optional[Tuple[float, float, float]], np.ndarray]:
        blurred = cv2.GaussianBlur(frame_bgr, (11, 11), 0)
        if self._hsv_buf is None or self._hsv_buf.shape != blurred.shape:
            self._hsv_buf = np.empty_like(blurred)
        cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV, dst=self._hsv_buf)
        mask = cv2.inRange(self._hsv_buf, self.hsv_lower, self.hsv_upper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        if not cnts:
            return None, mask
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        if radius < MIN_RADIUS_PX:
            return None, mask
        return (float(x), float(y), float(radius)), mask


def make_calibration_window() -> None:
    cv2.namedWindow("calibrate", cv2.WINDOW_NORMAL)
    for name, val, maxv in [
        ("H lo", YELLOW_LOWER_DEFAULT[0], 179),
        ("S lo", YELLOW_LOWER_DEFAULT[1], 255),
        ("V lo", YELLOW_LOWER_DEFAULT[2], 255),
        ("H hi", YELLOW_UPPER_DEFAULT[0], 179),
        ("S hi", YELLOW_UPPER_DEFAULT[1], 255),
        ("V hi", YELLOW_UPPER_DEFAULT[2], 255),
    ]:
        cv2.createTrackbar(name, "calibrate", val, maxv, lambda _v: None)


def read_calibration() -> Tuple[Tuple[int, int, int], Tuple[int, int, int]]:
    get = lambda n: cv2.getTrackbarPos(n, "calibrate")
    return ((get("H lo"), get("S lo"), get("V lo")), (get("H hi"), get("S hi"), get("V hi")))


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Tennis ball tracker -> reaction wheel PID")
    p.add_argument("--api", default="http://127.0.0.1:8000", help="FastAPI base URL")
    p.add_argument("--width", type=int, default=400, help="Resize frame width")
    p.add_argument("--dead-zone", type=float, default=0.05, help="Fraction of width considered centered")
    p.add_argument("--stop-radius", type=float, default=80.0, help="Radius (px) at which we brake")
    p.add_argument("--min-radius", type=float, default=float(MIN_RADIUS_PX), help="Reject smaller blobs")
    p.add_argument("--kp", type=float, default=0.08)
    p.add_argument("--ki", type=float, default=0.0)
    p.add_argument("--kd", type=float, default=0.02)
    p.add_argument("--max-rpm", type=float, default=float(MAX_ATTAINABLE_RPM),
                   help="Upper bound on |target_rpm| sent to /api/command")
    p.add_argument("--invert", action="store_true", help="Swap cw/ccw mapping")
    p.add_argument("--csi", action="store_true", help="Use Picamera2 instead of USB")
    p.add_argument("--src", type=int, default=0, help="USB camera index")
    p.add_argument("--rotate", type=int, default=180, choices=[0, 90, 180, 270],
                   help="Rotate frames (useful if the camera is mounted upside down)")
    p.add_argument("--dry-run", action="store_true", help="Print commands instead of POST")
    p.add_argument("--show", action="store_true", help="Show frame + mask window")
    p.add_argument("--calibrate", action="store_true", help="Open HSV trackbars")
    p.add_argument("--hsv-lower", type=parse_hsv, default=YELLOW_LOWER_DEFAULT, help='"H,S,V" lower bound')
    p.add_argument("--hsv-upper", type=parse_hsv, default=YELLOW_UPPER_DEFAULT, help='"H,S,V" upper bound')
    p.add_argument("--target-fps", type=float, default=30.0, help="Vision loop target FPS")
    return p


def main(argv: Optional[list[str]] = None) -> int:
    args = build_parser().parse_args(argv)

    cv2.setUseOptimized(True)
    try:
        cv2.setNumThreads(2)
    except cv2.error:
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

    def _open_csi():
        return CsiStream(rotate=args.rotate)

    def _open_usb():
        return VideoStream(src=args.src).start()

    def _probe(s, attempts: int = 25) -> Optional[np.ndarray]:
        for _ in range(attempts):
            frame = s.read()
            if frame is not None:
                return frame
            time.sleep(0.1)
        return None

    print(f">> opening camera ({'CSI' if args.csi else f'USB src={args.src}'})")
    stream = None
    probe = None
    try:
        stream = _open_csi() if args.csi else _open_usb()
    except Exception as exc:
        print(f">> ERROR: camera init failed: {exc}")
        if not args.csi:
            print("   Falling back to Picamera2 (CSI). Pass --csi next time to skip this.")
            try:
                stream = _open_csi()
            except Exception as exc2:
                print(f">> ERROR: Picamera2 fallback also failed: {exc2}")
                print("   Install with: sudo apt install -y python3-picamera2")
                return 3
        else:
            return 3

    time.sleep(1.5)
    probe = _probe(stream)

    # USB camera claimed /dev/video0 but produced no frames (common on Pi with CSI-only sensors).
    # Auto-fallback to Picamera2 before giving up.
    if probe is None and not args.csi:
        print(">> USB capture returned no frames; retrying via Picamera2 (CSI)...")
        try:
            stream.stop()
        except Exception:
            pass
        try:
            stream = _open_csi()
            time.sleep(1.5)
            probe = _probe(stream)
            if probe is not None:
                args.csi = True  # mark so the shutdown path knows
        except Exception as exc:
            print(f">> Picamera2 fallback failed: {exc}")

    if probe is None:
        try:
            if stream is not None:
                stream.stop()
        except Exception:
            pass
        src_desc = "CSI (picamera2)" if args.csi else f"/dev/video{args.src}"
        print(
            f">> ERROR: camera opened but returned no frames from {src_desc}.\n"
            f"   - USB: check `ls /dev/video*`; try --src 1 if another index.\n"
            f"   - Pi Camera over CSI: re-run with --csi; `sudo apt install -y python3-picamera2`.\n"
            f"   - If camera_stream.py works, run:  python follow_ball.py --csi --rotate 180 ..."
        )
        return 4
    print(f">> camera ready: {probe.shape[1]}x{probe.shape[0]} (rotate={args.rotate if args.csi else 0})")

    tracker = BallTracker(args.hsv_lower, args.hsv_upper)
    pid = PID(kp=args.kp, ki=args.ki, kd=args.kd)
    client = CommandClient(args.api, dry_run=args.dry_run)

    if args.calibrate:
        make_calibration_window()

    stopped_by_radius = False
    stop_hyst_px = args.stop_radius * 0.85

    running = True

    def handle_sigint(_sig, _frm):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, handle_sigint)
    signal.signal(signal.SIGTERM, handle_sigint)

    print(">> tracking; press Ctrl+C to quit (or 'q' in preview window)")
    frame_period = 1.0 / max(1.0, args.target_fps)
    last_t = time.time()
    last_print = 0.0

    try:
        while running:
            loop_start = time.time()
            raw = stream.read()
            if raw is None:
                time.sleep(0.01)
                continue

            frame = imutils.resize(raw, width=args.width)
            h, w = frame.shape[:2]
            center_x = w / 2.0
            dead_zone_px = args.dead_zone * w

            if args.calibrate:
                lo, hi = read_calibration()
                tracker.set_range(lo, hi)

            result, mask = tracker.detect(frame)

            now = time.time()
            dt = now - last_t
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

                if now - last_print > 0.5:
                    last_print = now
                    print(
                        f"[track] cx={cx:6.1f} err={cx - center_x:+7.1f} "
                        f"r={radius:5.1f} stop={stopped_by_radius}"
                    )

            if args.show:
                if result is not None:
                    cx, cy, radius = result
                    cv2.circle(frame, (int(cx), int(cy)), int(radius), (0, 255, 255), 2)
                    cv2.circle(frame, (int(cx), int(cy)), 3, (0, 0, 255), -1)
                cv2.line(frame, (int(center_x - dead_zone_px), 0),
                         (int(center_x - dead_zone_px), h), (80, 80, 80), 1)
                cv2.line(frame, (int(center_x + dead_zone_px), 0),
                         (int(center_x + dead_zone_px), h), (80, 80, 80), 1)
                cv2.imshow("frame", frame)
                cv2.imshow("mask", mask)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    running = False

            elapsed = time.time() - loop_start
            if elapsed < frame_period:
                time.sleep(frame_period - elapsed)
    finally:
        print(">> shutting down")
        try:
            client.brake(time.time())
        except Exception:
            pass
        try:
            stream.stop()
        except Exception:
            pass
        if args.show or args.calibrate:
            cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    sys.exit(main())
