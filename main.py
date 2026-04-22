import board
import busio
import math
import threading
import sys
from time import sleep, time
from gpiozero import PWMOutputDevice, DigitalOutputDevice, RotaryEncoder
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import BNO_REPORT_GYROSCOPE
from camera_stream import start_camera_stream 
# Camera stream:
# http://<pi-ip>:8000/
# http://<pi-ip>:8000/stream.mjpg

# --- 1. Pin Declarations (BCM GPIO) ---
RPWM_GPIO = 12  
LPWM_GPIO = 13  
R_EN_GPIO = 24  
L_EN_GPIO = 25  
ENC_A_GPIO = 22 
ENC_B_GPIO = 23 

# --- 2. Hardware Setup ---
rpwm = PWMOutputDevice(RPWM_GPIO, frequency=1000)
lpwm = PWMOutputDevice(LPWM_GPIO, frequency=1000)
r_en = DigitalOutputDevice(R_EN_GPIO)
l_en = DigitalOutputDevice(L_EN_GPIO)
encoder = RotaryEncoder(ENC_A_GPIO, ENC_B_GPIO, max_steps=0)

# --- 3. IMU Setup ---
try:
    i2c = board.I2C()
    bno = BNO08X_I2C(i2c)
    bno.enable_feature(BNO_REPORT_GYROSCOPE)
    imu_connected = True
    print(">> BNO085 IMU Online (45° Tilt Compensation Active)")
except Exception as e:
    print(f">> IMU Error: {e}")
    imu_connected = False

# --- 4. Constants & State ---
kp = 0.1
kd = 0.8  

TICKS_PER_REV = 360    
TILT_ANGLE = math.radians(45)

MAX_ATTAINABLE_RPM = 10
MAX_AUTO_RPM = 10
STABILITY_GAIN = 2
CAMERA_STREAM_PORT = 8000

# Global State
target_rpm = 0
current_dir = "brake"
system_mode = "manual" 
prev_error = 0
prev_time = time()
current_duty_cycle = 0

# --- 5. Support Functions ---

def get_velocity():
    global prev_time
    now = time()
    dt = now - prev_time
    
    if dt <= 0.02: 
        return 0, 0
    
    steps = encoder.steps
    encoder.steps = 0 
    
    rpm = (steps / TICKS_PER_REV) / (dt / 60.0)
    prev_time = now
    return rpm, dt

def motor_control_loop():
    """High-frequency PD loop for motor RPM control."""
    global prev_error, current_duty_cycle, target_rpm, current_dir, system_mode
    
    k = 0
    while True:
        current_rpm, dt = get_velocity()
        current_rpm = abs(current_rpm)

        if current_dir == "brake" or target_rpm == 0:
            rpwm.value = 0
            lpwm.value = 0
            current_duty_cycle = 0
            prev_error = 0 
        else:
            r_en.on()
            l_en.on()

            error = target_rpm - current_rpm
            derivative = (error - prev_error) / dt if dt > 0 else 0
            output = (kp * error) + (kd * derivative)
            
            current_duty_cycle += (output / 1000) 
            current_duty_cycle = max(0, min(1, current_duty_cycle))

            if current_dir == "ccw":
                rpwm.value = current_duty_cycle
                lpwm.value = 0
            elif current_dir == "cw":
                rpwm.value = 0
                lpwm.value = current_duty_cycle

            prev_error = error

        # Telemetry
        k += 1
        if k % 20 == 0:
            status = current_dir.upper() if target_rpm > 0 else "IDLE"
            true_yaw = 0.0
            if imu_connected:
                try:
                    gx, gy, gz = bno.gyro
                    # Calculate yaw relative to the tilted platform:
                    # $$true\_yaw = (g_z \cdot \cos(45^\circ)) - (g_y \cdot \sin(45^\circ))$$
                    true_yaw = (gz * math.cos(TILT_ANGLE)) - (gy * math.sin(TILT_ANGLE))
                except:
                    pass 

            print(f"[{status}] Mode: {system_mode.upper()} | Motor: {current_rpm:6.1f} RPM | Yaw: {true_yaw:6.3f} rad/s")
        
        sleep(0.05) 

def maintain_loop():
    """Active stabilization logic."""
    global target_rpm, current_dir, system_mode
    
    while True:
        if system_mode == "auto" and imu_connected:
            try:
                gx, gy, gz = bno.gyro
                true_yaw = (gz * math.cos(TILT_ANGLE)) - (gy * math.sin(TILT_ANGLE))
                
                if abs(true_yaw) > 1:  
                    rpm_req = abs(true_yaw * STABILITY_GAIN)
                    #print(rpm_req)
                    target_rpm = min(MAX_AUTO_RPM, rpm_req)
                    current_dir = "ccw" if true_yaw > 0 else "cw"
                else:
                    target_rpm = 0
                    current_dir = "brake"
            except:
                pass 
        sleep(0.02)

# --- 6. Main Entry Point ---

def main():
    global target_rpm, current_dir, system_mode, current_duty_cycle

    # Header
    print("\n" + "="*45)
    print("      REACTION WHEEL STABILIZATION SYSTEM")
    print("="*45)
    
    # 1. Selection Menu
    print("\nSelect Operating Mode:")
    print(" [1] Auto-Stabilize (IMU Active Correction)")
    print(" [2] Manual (Fixed RPM Commands)")
    
    while True:
        choice = input("\nEnter choice (1 or 2) > ").strip()
        if choice == "1":
            system_mode = "auto"
            print(">> INITIALIZING IN AUTO MODE...")
            break
        elif choice == "2":
            system_mode = "manual"
            print(">> INITIALIZING IN MANUAL MODE...")
            break
        else:
            print("Invalid selection. Please enter 1 or 2.")


    # 2. Start Background Threads
    motor_thread = threading.Thread(target=motor_control_loop, daemon=True)
    motor_thread.start()

    stabilizer_thread = threading.Thread(target=maintain_loop, daemon=True)
    stabilizer_thread.start()

    picam2 = start_camera_stream(port=CAMERA_STREAM_PORT)

    print("\n--- System Online ---")
    print("Commands: 'auto', 'manual', 'brake', or '[speed] [dir]' (e.g., '50 cw')")

    try:
        while True:
            cmd_input = input("\nCommand > ").strip().lower()

            if not cmd_input:
                continue

            if cmd_input == "brake":
                system_mode = "manual"
                target_rpm = 0
                current_dir = "brake"
                current_duty_cycle = 0
                print(">> Action: BRAKING")
            
            elif cmd_input == "auto":
                system_mode = "auto"
                current_duty_cycle = 0 # Reset PID for a clean transition
                print(f">> Action: SWITCHED TO AUTO (Gain: {STABILITY_GAIN})")
            
            elif cmd_input == "manual":
                system_mode = "manual"
                target_rpm = 0
                current_dir = "brake"
                print(">> Action: SWITCHED TO MANUAL (Idle)")
                
            else:
                try:
                    parts = cmd_input.split()
                    if len(parts) == 2:
                        speed = float(parts[0])
                        direction = parts[1]

                        if 0 <= speed <= MAX_ATTAINABLE_RPM and direction in ["cw", "ccw"]:
                            system_mode = "manual" 
                            target_rpm = speed
                            current_dir = direction
                            current_duty_cycle = 0 
                            print(f">> Action: MANUAL DRIVE {speed} RPM {direction.upper()}")
                        else:
                            print(f"Error: Speed must be 0-{MAX_ATTAINABLE_RPM} RPM.")
                    else:
                        print("Error: Unknown command. Use 'auto', 'manual', or '[speed] [dir]'.")
                except ValueError:
                    print("Error: Could not parse input.")

    except KeyboardInterrupt:
        print("\n\nUser interrupted. Shutting down...")
    finally:
        # Hardware Safety Shutdown
        picam2.stop_recording()
        rpwm.value = 0
        lpwm.value = 0
        r_en.off()
        l_en.off()
        print(">> GPIO cleaned up. Goodbye.")

if __name__ == "__main__":
    main()