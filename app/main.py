from __future__ import annotations

import asyncio
import threading
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Optional, Tuple

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, ConfigDict, field_validator

import camera_stream
from app.follow_loop import FollowController
from app.hardware import Hardware, create_hardware
from app.motor_loop import motor_control_loop
from app.state import MAX_ATTAINABLE_RPM, MotorState, TelemetryStore

ROOT = Path(__file__).resolve().parent.parent
DIST = ROOT / "web" / "dist"


@asynccontextmanager
async def lifespan(app: FastAPI):
    stop_event = threading.Event()
    motor_state = MotorState()
    telemetry = TelemetryStore()
    hw = create_hardware()
    thread = threading.Thread(
        target=motor_control_loop,
        args=(hw, motor_state, telemetry, stop_event),
        daemon=True,
        name="motor_control",
    )
    thread.start()
    follow = FollowController(motor_state)
    app.state.hw = hw
    app.state.motor_state = motor_state
    app.state.telemetry = telemetry
    app.state.stop_event = stop_event
    app.state.motor_thread = thread
    app.state.follow = follow
    try:
        yield
    finally:
        try:
            follow.stop(brake=True)
        except Exception:
            pass
        try:
            camera_stream.stop_camera_stream()
        except Exception:
            pass
        stop_event.set()
        thread.join(timeout=3.0)
        hw.shutdown()


app = FastAPI(title="Reaction wheel telemetry", lifespan=lifespan)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://127.0.0.1:5173", "http://localhost:5173"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


class CommandBody(BaseModel):
    """Plain strings avoid Literal validation 422s from odd client encodings; we validate in the handler."""

    model_config = ConfigDict(extra="ignore", str_strip_whitespace=True)

    action: Optional[str] = None
    target_rpm: Optional[float] = None
    direction: Optional[str] = None

    @field_validator("action", "direction", mode="before")
    @classmethod
    def only_str_or_none(cls, v: object) -> object:
        if v is None:
            return None
        if isinstance(v, str):
            return v
        return None

    @field_validator("target_rpm", mode="before")
    @classmethod
    def coerce_target_rpm(cls, v: object) -> object:
        if v is None or v == "":
            return None
        if isinstance(v, (int, float)):
            return float(v)
        if isinstance(v, str):
            try:
                return float(v.strip())
            except ValueError:
                return None
        return None


def _norm_action(raw: Optional[str]) -> Optional[str]:
    if not raw:
        return None
    s = raw.strip().lower()
    return s if s in ("brake", "stabilize") else None


def _norm_direction(raw: Optional[str]) -> Optional[str]:
    if not raw:
        return None
    s = raw.strip().lower()
    return s if s in ("cw", "ccw") else None


@app.post("/api/command")
async def post_command(body: CommandBody):
    motor_state: MotorState = app.state.motor_state
    hw: Hardware = app.state.hw
    action = _norm_action(body.action)
    direction = _norm_direction(body.direction)

    if action == "stabilize":
        if not getattr(hw, "imu_connected", False):
            return JSONResponse(
                {"ok": False, "error": "IMU not available; cannot run platform stabilize"},
                status_code=400,
            )
        motor_state.set_stabilize()
        return JSONResponse({"ok": True, "state": "stabilize"})
    if action == "brake":
        motor_state.set_brake()
        return JSONResponse({"ok": True, "state": "brake"})
    if body.target_rpm is not None and direction is not None:
        if not (0 <= body.target_rpm <= MAX_ATTAINABLE_RPM):
            return JSONResponse(
                {"ok": False, "error": f"target_rpm must be 0..{MAX_ATTAINABLE_RPM}"},
                status_code=400,
            )
        ok = motor_state.set_spin(body.target_rpm, direction)
        if not ok:
            return JSONResponse(
                {"ok": False, "error": "Invalid speed or direction"},
                status_code=400,
            )
        return JSONResponse(
            {"ok": True, "target_rpm": body.target_rpm, "direction": direction}
        )
    return JSONResponse(
        {
            "ok": False,
            "error": "Send action 'brake', 'stabilize', or target_rpm with direction (cw|ccw)",
        },
        status_code=400,
    )


def _follow_status_payload(follow: FollowController) -> dict:
    s = follow.status()
    cfg = s.config
    return {
        "running": s.running,
        "available": s.available,
        "error": s.error,
        "fps": round(s.fps, 1),
        "detected": s.detected,
        "cx": s.cx,
        "cy": s.cy,
        "radius": s.radius,
        "error_px": s.error_px,
        "target_rpm": s.target_rpm,
        "direction": s.direction,
        "config": {
            "kp": cfg.kp,
            "ki": cfg.ki,
            "kd": cfg.kd,
            "max_rpm": cfg.max_rpm,
            "dead_zone": cfg.dead_zone,
            "stop_radius": cfg.stop_radius,
            "invert": cfg.invert,
            "hsv_lower": list(cfg.hsv_lower),
            "hsv_upper": list(cfg.hsv_upper),
            "track_width": cfg.track_width,
        },
    }


@app.websocket("/ws/telemetry")
async def websocket_telemetry(websocket: WebSocket):
    await websocket.accept()
    telemetry: TelemetryStore = app.state.telemetry
    follow: FollowController = app.state.follow
    try:
        while True:
            snap = telemetry.read()
            await websocket.send_json(
                {
                    "t": snap.t,
                    "motor_rpm": snap.motor_rpm,
                    "wheel_rad_s": snap.wheel_rad_s,
                    "imu_ok": snap.imu_ok,
                    "duty_cycle": snap.duty_cycle,
                    "target_rpm": snap.target_rpm,
                    "direction": snap.direction,
                    "stabilize_active": snap.stabilize_active,
                    "stabilize_pid_u": snap.stabilize_pid_u,
                    "follow": _follow_status_payload(follow),
                }
            )
            await asyncio.sleep(0.05)
    except WebSocketDisconnect:
        return


@app.get("/api/health")
async def health():
    hw: Hardware = app.state.hw
    return {
        "ok": True,
        "imu_connected": getattr(hw, "imu_connected", False),
        "mock": hw.__class__.__name__ == "MockHardware",
        "camera_available": camera_stream.is_available(),
        "camera_running": camera_stream.is_running(),
    }


# ---------------------------------------------------------------------------
# Camera + ball-follow endpoints
# ---------------------------------------------------------------------------


def _hsv_tuple(v: object) -> Optional[Tuple[int, int, int]]:
    if v is None:
        return None
    if isinstance(v, str):
        parts = [p.strip() for p in v.split(",")]
        if len(parts) != 3:
            return None
        try:
            return (int(parts[0]), int(parts[1]), int(parts[2]))
        except ValueError:
            return None
    if isinstance(v, (list, tuple)) and len(v) == 3:
        try:
            return (int(v[0]), int(v[1]), int(v[2]))
        except (TypeError, ValueError):
            return None
    return None


class FollowConfigBody(BaseModel):
    model_config = ConfigDict(extra="ignore")
    kp: Optional[float] = None
    ki: Optional[float] = None
    kd: Optional[float] = None
    max_rpm: Optional[float] = None
    dead_zone: Optional[float] = None
    stop_radius: Optional[float] = None
    invert: Optional[bool] = None
    hsv_lower: Optional[object] = None
    hsv_upper: Optional[object] = None
    track_width: Optional[int] = None


def _coerce_follow_kwargs(body: FollowConfigBody) -> dict:
    kwargs: dict = {}
    for name in ("kp", "ki", "kd", "max_rpm", "dead_zone", "stop_radius", "track_width", "invert"):
        v = getattr(body, name)
        if v is not None:
            kwargs[name] = v
    lower = _hsv_tuple(body.hsv_lower)
    upper = _hsv_tuple(body.hsv_upper)
    if lower is not None:
        kwargs["hsv_lower"] = lower
    if upper is not None:
        kwargs["hsv_upper"] = upper
    if "max_rpm" in kwargs:
        kwargs["max_rpm"] = float(min(float(MAX_ATTAINABLE_RPM), max(0.0, float(kwargs["max_rpm"]))))
    if "dead_zone" in kwargs:
        kwargs["dead_zone"] = float(max(0.0, min(0.5, float(kwargs["dead_zone"]))))
    if "stop_radius" in kwargs:
        kwargs["stop_radius"] = float(max(0.0, float(kwargs["stop_radius"])))
    return kwargs


@app.get("/api/follow/status")
async def follow_status():
    follow: FollowController = app.state.follow
    return _follow_status_payload(follow)


@app.post("/api/follow/start")
async def follow_start(body: FollowConfigBody):
    follow: FollowController = app.state.follow
    kwargs = _coerce_follow_kwargs(body)
    ok, err = follow.start(**kwargs)
    if not ok:
        return JSONResponse(
            {"ok": False, "error": err or "Failed to start follow loop", **_follow_status_payload(follow)},
            status_code=503,
        )
    return {"ok": True, **_follow_status_payload(follow)}


@app.post("/api/follow/stop")
async def follow_stop():
    follow: FollowController = app.state.follow
    follow.stop(brake=True)
    return {"ok": True, **_follow_status_payload(follow)}


@app.post("/api/follow/config")
async def follow_config(body: FollowConfigBody):
    follow: FollowController = app.state.follow
    kwargs = _coerce_follow_kwargs(body)
    follow.update(**kwargs)
    return {"ok": True, **_follow_status_payload(follow)}


@app.get("/api/camera/stream.mjpg")
async def camera_stream_mjpg():
    if not camera_stream.is_available():
        return JSONResponse(
            {
                "ok": False,
                "error": "Camera dependencies (picamera2/cv2) unavailable on this host.",
            },
            status_code=503,
        )

    if not camera_stream.start_capture_only():
        return JSONResponse(
            {"ok": False, "error": "Failed to start Picamera2 capture."},
            status_code=503,
        )

    boundary = "FRAME"

    async def gen():
        deadline_first_frame = asyncio.get_event_loop().time() + 5.0
        try:
            while True:
                jpeg = camera_stream.get_latest_jpeg()
                if jpeg is None:
                    if asyncio.get_event_loop().time() > deadline_first_frame:
                        break
                    await asyncio.sleep(0.05)
                    continue
                deadline_first_frame = asyncio.get_event_loop().time() + 5.0
                yield (
                    f"--{boundary}\r\n"
                    f"Content-Type: image/jpeg\r\n"
                    f"Content-Length: {len(jpeg)}\r\n\r\n"
                ).encode("ascii") + jpeg + b"\r\n"
                await asyncio.sleep(1.0 / 30.0)
        except (asyncio.CancelledError, BrokenPipeError, ConnectionResetError):
            return

    return StreamingResponse(
        gen(),
        media_type=f"multipart/x-mixed-replace; boundary={boundary}",
        headers={"Cache-Control": "no-cache, private", "Pragma": "no-cache"},
    )


@app.get("/api/camera/snapshot.jpg")
async def camera_snapshot():
    """One-shot JPEG snapshot, useful as a fallback when MJPEG can't connect."""
    if not camera_stream.is_available():
        return JSONResponse(
            {"ok": False, "error": "Camera unavailable on this host."},
            status_code=503,
        )
    if not camera_stream.start_capture_only():
        return JSONResponse(
            {"ok": False, "error": "Failed to start Picamera2 capture."},
            status_code=503,
        )
    for _ in range(50):
        jpeg = camera_stream.get_latest_jpeg()
        if jpeg is not None:
            return StreamingResponse(iter([jpeg]), media_type="image/jpeg")
        await asyncio.sleep(0.05)
    return JSONResponse({"ok": False, "error": "No frame yet."}, status_code=504)


if DIST.exists():
    app.mount("/", StaticFiles(directory=str(DIST), html=True), name="spa")
else:

    @app.get("/")
    async def root_placeholder():
        return JSONResponse(
            {
                "message": "Web UI not built. Run: cd web && npm install && npm run build",
                "telemetry_ws": "/ws/telemetry",
            }
        )
