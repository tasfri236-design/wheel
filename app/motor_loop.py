from __future__ import annotations

import threading
from time import sleep, time

from app.hardware import Hardware, MockHardware, PiHardware
from app.state import (
    MAX_ATTAINABLE_RPM,
    STABILIZE_DEBOUNCE_COUNT,
    STABILIZE_INTEGRAL_MAX,
    STABILIZE_KD,
    STABILIZE_KI,
    STABILIZE_KP,
    STABILIZE_THRESHOLD_RAD_S,
    STABILIZE_TORQUE_SIGN,
    MotorState,
    TelemetryStore,
)

kp = 0.8
kd = 0.2
DT_OUTER = 0.05


def motor_control_loop(
    hw: Hardware,
    motor_state: MotorState,
    telemetry: TelemetryStore,
    stop_event: threading.Event,
) -> None:
    prev_error = 0.0
    current_duty_cycle = 0.0
    prev_cmd: tuple[float, str] = (-1.0, "")
    prev_eff_stab = False

    outer_int_e = 0.0
    outer_prev_e = 0.0
    deb_below = 0
    pid_u = 0.0

    while not stop_event.is_set():
        omega, imu_ok = hw.read_wheel_rad_s()
        stabilize = motor_state.stabilize_active()
        omega_valid = imu_ok and omega is not None

        if stabilize and not omega_valid:
            motor_state.set_brake()
            outer_int_e = 0.0
            outer_prev_e = 0.0
            deb_below = 0
            pid_u = 0.0

        stabilize = motor_state.stabilize_active()

        target_rpm: float
        current_dir: str

        if stabilize and omega_valid:
            wplat = float(omega)  # omega_valid: IMU sample present
            if abs(wplat) < STABILIZE_THRESHOLD_RAD_S:
                deb_below += 1
            else:
                deb_below = 0

            if deb_below >= STABILIZE_DEBOUNCE_COUNT:
                motor_state.set_brake()
                deb_below = 0
                outer_int_e = 0.0
                outer_prev_e = 0.0
                pid_u = 0.0
                target_rpm, current_dir = 0.0, "brake"
            else:
                e = 0.0 - wplat
                outer_int_e += e * DT_OUTER
                outer_int_e = max(-STABILIZE_INTEGRAL_MAX, min(STABILIZE_INTEGRAL_MAX, outer_int_e))
                de = (e - outer_prev_e) / DT_OUTER
                outer_prev_e = e
                pid_u = STABILIZE_KP * e + STABILIZE_KI * outer_int_e + STABILIZE_KD * de

                signed = pid_u * STABILIZE_TORQUE_SIGN
                if abs(signed) < 1e-6:
                    target_rpm, current_dir = 0.0, "brake"
                else:
                    current_dir = "cw" if signed > 0 else "ccw"
                    target_rpm = min(MAX_ATTAINABLE_RPM, abs(signed))

                if target_rpm >= MAX_ATTAINABLE_RPM - 1e-6:
                    outer_int_e -= e * DT_OUTER * 0.5
        else:
            deb_below = 0
            if not stabilize:
                outer_int_e = 0.0
                outer_prev_e = 0.0
                pid_u = 0.0
            target_rpm, current_dir = motor_state.get()

        stabilize = motor_state.stabilize_active()
        eff_stab = stabilize and imu_ok and omega is not None

        if prev_eff_stab and not eff_stab:
            current_duty_cycle = 0.0
            prev_error = 0.0

        if eff_stab:
            if not prev_eff_stab:
                current_duty_cycle = 0.0
                prev_error = 0.0
        else:
            cmd = (target_rpm, current_dir)
            if cmd != prev_cmd:
                current_duty_cycle = 0.0
                prev_error = 0.0
                prev_cmd = cmd

        prev_eff_stab = eff_stab

        current_rpm, dt = hw.get_velocity()
        current_rpm = abs(current_rpm)

        if isinstance(hw, PiHardware):
            if current_dir == "brake" or target_rpm == 0:
                hw.rpwm.value = 0
                hw.lpwm.value = 0
                current_duty_cycle = 0.0
                prev_error = 0.0
            else:
                hw.r_en.on()
                hw.l_en.on()
                error = target_rpm - current_rpm
                derivative = (error - prev_error) / dt if dt > 0 else 0.0
                output = (kp * error) + (kd * derivative)
                current_duty_cycle += output / 1000.0
                current_duty_cycle = max(0.0, min(1.0, current_duty_cycle))
                if current_dir == "cw":
                    hw.rpwm.value = current_duty_cycle
                    hw.lpwm.value = 0.0
                elif current_dir == "ccw":
                    hw.rpwm.value = 0.0
                    hw.lpwm.value = current_duty_cycle
                prev_error = error
        elif isinstance(hw, MockHardware):
            if current_dir == "brake" or target_rpm == 0:
                current_duty_cycle = 0.0
                prev_error = 0.0
            else:
                error = target_rpm - current_rpm
                derivative = (error - prev_error) / dt if dt > 0 else 0.0
                output = (kp * error) + (kd * derivative)
                current_duty_cycle += output / 1000.0
                current_duty_cycle = max(0.0, min(1.0, current_duty_cycle))
                prev_error = error

        if isinstance(hw, MockHardware):
            hw.step_platform(dt if dt > 0 else DT_OUTER, current_duty_cycle, current_dir)

        telemetry.update(
            t=time(),
            motor_rpm=current_rpm,
            wheel_rad_s=omega,
            imu_ok=imu_ok,
            duty_cycle=current_duty_cycle,
            target_rpm=target_rpm,
            direction=current_dir,
            stabilize_active=motor_state.stabilize_active(),
            stabilize_pid_u=pid_u,
        )

        sleep(DT_OUTER)
