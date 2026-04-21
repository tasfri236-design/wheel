from __future__ import annotations

import math
import os
import random
from abc import ABC, abstractmethod
from time import time
from typing import Optional, Tuple

TICKS_PER_REV = 360
TILT_ANGLE = math.radians(45)


def _mock_enabled() -> bool:
    v = os.environ.get("MOCK_TELEMETRY", "").lower()
    return v in ("1", "true", "yes")


class Hardware(ABC):
    imu_connected: bool

    @abstractmethod
    def get_velocity(self) -> Tuple[float, float]:
        """Return (rpm_signed_or_raw, dt_seconds)."""

    @abstractmethod
    def read_wheel_rad_s(self) -> Tuple[Optional[float], bool]:
        """Return (yaw rate rad/s or None, imu_read_ok)."""

    @abstractmethod
    def shutdown(self) -> None:
        ...


class MockHardware(Hardware):
    """Synthetic motor RPM plus a simple platform yaw plant for PID stabilize testing."""

    def __init__(self) -> None:
        self.imu_connected = True
        self._prev_time = time()
        self._angle = 0.0
        self._omega_platform = 0.45

    def get_velocity(self) -> Tuple[float, float]:
        now = time()
        dt = now - self._prev_time
        if dt <= 0.02:
            return 0.0, dt
        self._prev_time = now
        self._angle += 0.15 * dt
        base = 25 + 20 * math.sin(self._angle)
        noise = random.uniform(-2, 2)
        rpm = base + noise
        return rpm, dt

    def read_wheel_rad_s(self) -> Tuple[Optional[float], bool]:
        return float(self._omega_platform), True

    def step_platform(self, dt: float, duty: float, direction: str) -> None:
        if direction == "cw":
            tau = duty
        elif direction == "ccw":
            tau = -duty
        else:
            tau = 0.0
        tau *= 1.1
        wdot = 2.2 * tau - 0.55 * self._omega_platform
        self._omega_platform += wdot * dt
        self._omega_platform += 0.02 * random.uniform(-1.0, 1.0)
        self._omega_platform = max(-2.5, min(2.5, self._omega_platform))

    def shutdown(self) -> None:
        pass


class PiHardware(Hardware):
    def __init__(self) -> None:
        from gpiozero import PWMOutputDevice, DigitalOutputDevice, RotaryEncoder
        from adafruit_bno08x.i2c import BNO08X_I2C
        from adafruit_bno08x import BNO_REPORT_GYROSCOPE
        import board

        self.RPWM_GPIO = 12
        self.LPWM_GPIO = 13
        self.R_EN_GPIO = 24
        self.L_EN_GPIO = 25
        self.ENC_A_GPIO = 22
        self.ENC_B_GPIO = 23

        self.rpwm = PWMOutputDevice(self.RPWM_GPIO, frequency=1000)
        self.lpwm = PWMOutputDevice(self.LPWM_GPIO, frequency=1000)
        self.r_en = DigitalOutputDevice(self.R_EN_GPIO)
        self.l_en = DigitalOutputDevice(self.L_EN_GPIO)
        self.encoder = RotaryEncoder(self.ENC_A_GPIO, self.ENC_B_GPIO, max_steps=0)

        self.bno: Optional[object] = None
        self.imu_connected = False
        try:
            i2c = board.I2C()
            self.bno = BNO08X_I2C(i2c)
            self.bno.enable_feature(BNO_REPORT_GYROSCOPE)
            self.imu_connected = True
            print(">> BNO085 IMU Online (45° tilt compensation active)")
        except Exception as e:
            print(f">> IMU error: {e}")

        self._prev_time = time()

    def get_velocity(self) -> Tuple[float, float]:
        now = time()
        dt = now - self._prev_time
        if dt <= 0.02:
            return 0.0, 0.0
        steps = self.encoder.steps
        self.encoder.steps = 0
        rpm = (steps / TICKS_PER_REV) / (dt / 60.0)
        self._prev_time = now
        return rpm, dt

    def read_wheel_rad_s(self) -> Tuple[Optional[float], bool]:
        if not self.imu_connected or self.bno is None:
            return None, False
        try:
            gx, gy, gz = self.bno.gyro
            true_yaw = (gz * math.cos(TILT_ANGLE)) - (gy * math.sin(TILT_ANGLE))
            return float(true_yaw), True
        except Exception:
            return None, False

    def shutdown(self) -> None:
        self.rpwm.value = 0
        self.lpwm.value = 0
        self.r_en.off()
        self.l_en.off()


def create_hardware() -> Hardware:
    if _mock_enabled():
        print(">> MOCK_TELEMETRY: synthetic encoder + IMU data")
        return MockHardware()
    try:
        return PiHardware()
    except Exception as e:
        print(f">> Hardware init failed ({e}); set MOCK_TELEMETRY=1 for dev without GPIO.")
        raise
