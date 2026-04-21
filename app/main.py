from __future__ import annotations

import asyncio
import threading
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Optional

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, ConfigDict, field_validator

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
    app.state.hw = hw
    app.state.motor_state = motor_state
    app.state.telemetry = telemetry
    app.state.stop_event = stop_event
    app.state.motor_thread = thread
    yield
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


@app.websocket("/ws/telemetry")
async def websocket_telemetry(websocket: WebSocket):
    await websocket.accept()
    telemetry: TelemetryStore = app.state.telemetry
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
    }


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
