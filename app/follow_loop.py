"""Camera-driven ball-follow PID loop, owned by the FastAPI process.

Reuses ``BallTracker`` and ``PID`` from ``follow_ball.py`` so the math/HSV
pipeline is identical to the standalone CLI script. Frames come from
``camera_stream`` (same Picamera2 pipeline that powers ``camera_viewer.py``).

Output is written directly to the existing :class:`MotorState`, so the motor
control thread keeps full ownership of GPIO/PWM. We never self-call HTTP.
"""
from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from typing import Optional, Tuple

import camera_stream
from app.state import MAX_ATTAINABLE_RPM, MotorState

# Lazy imports for opencv stack — keeps FastAPI importable on machines without
# OpenCV installed (e.g. fresh dev Macs). Resolved inside ``start()``.
BallTracker = None  # type: ignore[assignment]
PID = None  # type: ignore[assignment]
imutils = None  # type: ignore[assignment]


YELLOW_LOWER_DEFAULT = (25, 80, 80)
YELLOW_UPPER_DEFAULT = (45, 255, 255)

DEFAULT_KP = 0.08
DEFAULT_KI = 0.0
DEFAULT_KD = 0.02
DEFAULT_MAX_RPM = 60.0
DEFAULT_DEAD_ZONE = 0.05
DEFAULT_STOP_RADIUS = 80.0
DEFAULT_TRACK_WIDTH = 400

COMMAND_MIN_INTERVAL = 1.0 / 20.0  # 20 Hz outbound command rate
COMMAND_RPM_EPSILON = 2.0
NO_FRAME_BRAKE_AFTER_S = 1.0


@dataclass
class FollowConfig:
    kp: float = DEFAULT_KP
    ki: float = DEFAULT_KI
    kd: float = DEFAULT_KD
    max_rpm: float = DEFAULT_MAX_RPM
    dead_zone: float = DEFAULT_DEAD_ZONE
    stop_radius: float = DEFAULT_STOP_RADIUS
    invert: bool = False
    hsv_lower: Tuple[int, int, int] = YELLOW_LOWER_DEFAULT
    hsv_upper: Tuple[int, int, int] = YELLOW_UPPER_DEFAULT
    track_width: int = DEFAULT_TRACK_WIDTH


@dataclass
class FollowStatus:
    running: bool = False
    available: bool = True
    error: Optional[str] = None
    fps: float = 0.0
    detected: bool = False
    cx: Optional[float] = None
    cy: Optional[float] = None
    radius: Optional[float] = None
    error_px: Optional[float] = None
    target_rpm: float = 0.0
    direction: str = "brake"
    config: FollowConfig = field(default_factory=FollowConfig)


def _ensure_cv_imports() -> Optional[str]:
    """Import opencv-stack lazily. Returns an error string or None on success."""
    global BallTracker, PID, imutils
    if BallTracker is not None and PID is not None and imutils is not None:
        return None
    try:
        import imutils as _imutils  # noqa: F401
        from follow_ball import BallTracker as _BallTracker, PID as _PID
    except Exception as exc:  # pragma: no cover - environment dependent
        return f"OpenCV stack unavailable: {exc}"
    BallTracker = _BallTracker
    PID = _PID
    imutils = _imutils
    return None


class FollowController:
    """Owns the follow thread and exposes start/stop/update/status."""

    def __init__(self, motor_state: MotorState) -> None:
        self._motor = motor_state
        self._lock = threading.Lock()
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._cfg = FollowConfig()
        self._status = FollowStatus(running=False, available=camera_stream.is_available())

    # ---- public API -----------------------------------------------------

    def is_running(self) -> bool:
        with self._lock:
            return self._status.running

    def status(self) -> FollowStatus:
        with self._lock:
            return FollowStatus(
                running=self._status.running,
                available=self._status.available,
                error=self._status.error,
                fps=self._status.fps,
                detected=self._status.detected,
                cx=self._status.cx,
                cy=self._status.cy,
                radius=self._status.radius,
                error_px=self._status.error_px,
                target_rpm=self._status.target_rpm,
                direction=self._status.direction,
                config=FollowConfig(**self._cfg.__dict__),
            )

    def update(self, **fields) -> FollowConfig:
        """Hot-update a subset of config fields. Safe to call while running."""
        with self._lock:
            for k, v in fields.items():
                if hasattr(self._cfg, k) and v is not None:
                    setattr(self._cfg, k, v)
            return FollowConfig(**self._cfg.__dict__)

    def start(self, **fields) -> Tuple[bool, Optional[str]]:
        """Start the follow loop. Returns (ok, error_message)."""
        if not camera_stream.is_available():
            with self._lock:
                self._status.available = False
                self._status.error = (
                    "picamera2/cv2 not installed on this host. "
                    "Install with: sudo apt install -y python3-picamera2"
                )
            return False, self._status.error

        err = _ensure_cv_imports()
        if err is not None:
            with self._lock:
                self._status.error = err
            return False, err

        if not camera_stream.start_capture_only():
            with self._lock:
                self._status.error = "Failed to start Picamera2 capture."
            return False, self._status.error

        with self._lock:
            for k, v in fields.items():
                if hasattr(self._cfg, k) and v is not None:
                    setattr(self._cfg, k, v)
            if self._status.running:
                self._status.error = None
                return True, None
            self._stop_event.clear()
            self._status = FollowStatus(
                running=True,
                available=True,
                error=None,
                config=FollowConfig(**self._cfg.__dict__),
            )

        thread = threading.Thread(target=self._run, name="follow_loop", daemon=True)
        with self._lock:
            self._thread = thread
        thread.start()
        return True, None

    def stop(self, brake: bool = True) -> None:
        with self._lock:
            thread = self._thread
        if thread is None:
            return
        self._stop_event.set()
        thread.join(timeout=2.0)
        with self._lock:
            self._thread = None
            self._status.running = False
            self._status.detected = False
            self._status.fps = 0.0
            self._status.target_rpm = 0.0
            self._status.direction = "brake"
        if brake:
            self._motor.set_brake()

    # ---- internal -------------------------------------------------------

    def _run(self) -> None:
        assert BallTracker is not None and PID is not None and imutils is not None  # noqa: S101
        with self._lock:
            cfg = FollowConfig(**self._cfg.__dict__)

        tracker = BallTracker(cfg.hsv_lower, cfg.hsv_upper)
        pid = PID(kp=cfg.kp, ki=cfg.ki, kd=cfg.kd)

        last_command_t = 0.0
        last_target_rpm = 0.0
        last_direction = "brake"
        last_loop_t = time.monotonic()
        ema_dt = 1.0 / 30.0

        last_frame_seen_t = time.monotonic()
        stopped_by_radius = False
        last_no_frame_warn = 0.0

        try:
            while not self._stop_event.is_set():
                # Refresh config (allows hot updates).
                with self._lock:
                    cfg = FollowConfig(**self._cfg.__dict__)
                pid.kp, pid.ki, pid.kd = cfg.kp, cfg.ki, cfg.kd
                tracker.set_range(cfg.hsv_lower, cfg.hsv_upper)

                raw = camera_stream.get_latest_bgr_frame()
                now = time.monotonic()

                if raw is None:
                    if now - last_frame_seen_t > NO_FRAME_BRAKE_AFTER_S:
                        if last_direction != "brake":
                            self._motor.set_brake()
                            last_direction = "brake"
                            last_target_rpm = 0.0
                        if now - last_no_frame_warn > 5.0:
                            last_no_frame_warn = now
                        with self._lock:
                            self._status.detected = False
                            self._status.fps = 0.0
                            self._status.target_rpm = 0.0
                            self._status.direction = "brake"
                    time.sleep(0.02)
                    continue

                last_frame_seen_t = now

                frame = imutils.resize(raw, width=cfg.track_width)
                h, w = frame.shape[:2]
                center_x = w // 2
                dead_zone_px = int(round(cfg.dead_zone * w))
                stop_hyst_px = cfg.stop_radius * 0.85

                dt = max(1e-3, now - last_loop_t)
                last_loop_t = now
                ema_dt = 0.9 * ema_dt + 0.1 * dt

                detection, _mask = tracker.detect(frame)

                send_brake = False
                if detection is None:
                    send_brake = True
                    pid.reset()
                    cx = cy = radius = error_px = None
                else:
                    cx, cy, radius = detection
                    error_px = cx - center_x
                    if radius >= cfg.stop_radius:
                        stopped_by_radius = True
                    elif stopped_by_radius and radius < stop_hyst_px:
                        stopped_by_radius = False

                    if stopped_by_radius or abs(error_px) <= dead_zone_px:
                        send_brake = True
                        pid.reset()

                target_rpm = 0.0
                direction = "brake"

                if not send_brake and detection is not None:
                    u = pid.step(error_px, dt)
                    positive_right = (u > 0)
                    if cfg.invert:
                        positive_right = not positive_right
                    direction = "cw" if positive_right else "ccw"
                    target_rpm = min(
                        float(cfg.max_rpm),
                        float(MAX_ATTAINABLE_RPM),
                        abs(u),
                    )

                # Rate-limit + dedup commands.
                command_dt = now - last_command_t
                wants_brake = send_brake or target_rpm < 1.0
                if wants_brake:
                    if last_direction != "brake" and command_dt >= COMMAND_MIN_INTERVAL:
                        self._motor.set_brake()
                        last_direction = "brake"
                        last_target_rpm = 0.0
                        last_command_t = now
                else:
                    rpm_changed = abs(target_rpm - last_target_rpm) >= COMMAND_RPM_EPSILON
                    dir_changed = direction != last_direction
                    if (rpm_changed or dir_changed) and command_dt >= COMMAND_MIN_INTERVAL:
                        ok = self._motor.set_spin(target_rpm, direction)
                        if ok:
                            last_target_rpm = target_rpm
                            last_direction = direction
                            last_command_t = now

                with self._lock:
                    self._status.fps = (1.0 / ema_dt) if ema_dt > 0 else 0.0
                    self._status.detected = detection is not None
                    self._status.cx = float(cx) if cx is not None else None
                    self._status.cy = float(cy) if cy is not None else None
                    self._status.radius = float(radius) if radius is not None else None
                    self._status.error_px = float(error_px) if error_px is not None else None
                    self._status.target_rpm = float(last_target_rpm)
                    self._status.direction = last_direction

                # Pace the loop a touch so we don't burn CPU on duplicate frames.
                time.sleep(0.005)
        except Exception as exc:  # pragma: no cover - defensive
            with self._lock:
                self._status.error = f"follow loop crashed: {exc}"
            try:
                self._motor.set_brake()
            except Exception:
                pass
        finally:
            with self._lock:
                self._status.running = False
                self._status.detected = False
                self._status.fps = 0.0
                self._status.target_rpm = 0.0
                self._status.direction = "brake"
            try:
                self._motor.set_brake()
            except Exception:
                pass
