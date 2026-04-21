from __future__ import annotations

import os
from dataclasses import dataclass, replace
from threading import Lock
from typing import Optional

MAX_ATTAINABLE_RPM = 100

STABILIZE_THRESHOLD_RAD_S = float(os.environ.get("STABILIZE_THRESHOLD_RAD_S", "0.2"))
STABILIZE_DEBOUNCE_COUNT = int(os.environ.get("STABILIZE_DEBOUNCE_COUNT", "15"))
STABILIZE_KP = float(os.environ.get("STABILIZE_KP", "12.0"))
STABILIZE_KI = float(os.environ.get("STABILIZE_KI", "2.0"))
STABILIZE_KD = float(os.environ.get("STABILIZE_KD", "0.08"))
STABILIZE_TORQUE_SIGN = int(os.environ.get("STABILIZE_TORQUE_SIGN", "1"))
STABILIZE_INTEGRAL_MAX = float(os.environ.get("STABILIZE_INTEGRAL_MAX", "5.0"))


@dataclass(frozen=True)
class TelemetrySnapshot:
    t: float
    motor_rpm: float
    wheel_rad_s: Optional[float]
    imu_ok: bool
    duty_cycle: float
    target_rpm: float
    direction: str
    stabilize_active: bool
    stabilize_pid_u: float


class TelemetryStore:
    def __init__(self) -> None:
        self._lock = Lock()
        self._snap = TelemetrySnapshot(
            t=0.0,
            motor_rpm=0.0,
            wheel_rad_s=None,
            imu_ok=False,
            duty_cycle=0.0,
            target_rpm=0.0,
            direction="brake",
            stabilize_active=False,
            stabilize_pid_u=0.0,
        )

    def update(self, **kwargs) -> None:
        with self._lock:
            self._snap = replace(self._snap, **kwargs)

    def read(self) -> TelemetrySnapshot:
        with self._lock:
            return self._snap


class MotorState:
    def __init__(self) -> None:
        self._lock = Lock()
        self._target_rpm = 0.0
        self._current_dir = "brake"
        self._stabilize_active = False

    def stabilize_active(self) -> bool:
        with self._lock:
            return self._stabilize_active

    def set_stabilize(self) -> None:
        with self._lock:
            self._stabilize_active = True

    def set_brake(self) -> None:
        with self._lock:
            self._target_rpm = 0.0
            self._current_dir = "brake"
            self._stabilize_active = False

    def set_spin(self, target_rpm: float, direction: str) -> bool:
        if not (0 <= target_rpm <= MAX_ATTAINABLE_RPM and direction in ("cw", "ccw")):
            return False
        with self._lock:
            self._target_rpm = float(target_rpm)
            self._current_dir = direction
            self._stabilize_active = False
        return True

    def get(self) -> tuple[float, str]:
        with self._lock:
            return self._target_rpm, self._current_dir
