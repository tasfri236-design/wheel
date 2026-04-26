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
import cv2
import imutils
import numpy as np 
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
kp = 0.4
kd = 0.4  

TICKS_PER_REV = 360    
TILT_ANGLE = math.radians(45)

MAX_ATTAINABLE_RPM = 10
MAX_AUTO_RPM = 10
STABILITY_GAIN = 2
CAMERA_STREAM_PORT = 8000

# --- Ball Tracking Constants ---
BALL_TRACKING_FPS = 30
BALL_LOWER_HSV = np.array([25, 50, 50], dtype=np.uint8)    # Yellow lower bound
BALL_UPPER_HSV = np.array([85, 255, 255], dtype=np.uint8)  # Yellow upper bound
BALL_MIN_RADIUS_PX = 8
BALL_STOP_RADIUS_PX = 80.0
BALL_DEAD_ZONE_FRACTION = 0.05
BALL_KP = 0.08
BALL_KI = 0.0
BALL_KD = 0.02
BALL_MAX_RPM = 10

# Global State
target_rpm = 0
current_dir = "brake"
system_mode = "manual" 
prev_error = 0
prev_time = time()
current_duty_cycle = 0
ball_tracking_running = False
ball_frame_buffer = None

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

def detect_tennis_ball(frame_bgr):
    
    """Detect yellow tennis ball and return (cx, cy, radius) or None."""
    blurred = cv2.GaussianBlur(frame_bgr, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, BALL_LOWER_HSV, BALL_UPPER_HSV)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    
    cv2.imshow("mask", mask)
    cv2.waitKey(1)

    if not cnts:
        return None
    
    c = max(cnts, key=cv2.contourArea)
    ((x, y), radius) = cv2.minEnclosingCircle(c)
    
    if radius < BALL_MIN_RADIUS_PX:
        return None
    
    return (float(x), float(y), float(radius))

def ball_tracking_loop():
    """Tennis ball tracking loop that adjusts wheel based on ball position."""
    global target_rpm, current_dir, system_mode, ball_tracking_running
    
    try:
        # Open camera feed
        from imutils.video import VideoStream
        print(">> Opening camera for ball tracking...")
        stream = VideoStream(src=0).start()
        sleep(1.5)
        
        # PID controller for tracking
        pid_integral = 0.0
        pid_prev_error = None
        
        stopped_by_radius = False
        stop_hyst_px = BALL_STOP_RADIUS_PX * 0.85
        last_print = time()
        last_command_time = time()
        
        print(">> Ball tracking started. Press Ctrl+C to stop.")
        
        while ball_tracking_running:
            loop_start = time()
            raw = stream.read()
            
            if raw is None:
                sleep(0.01)
                continue
            
            frame = imutils.resize(raw, width=400)
            h, w = frame.shape[:2]
            center_x = w / 2.0
            dead_zone_px = BALL_DEAD_ZONE_FRACTION * w
            
            result = detect_tennis_ball(frame)
            now = time()
            
            if result is None:
                # No ball detected - brake
                target_rpm = 0
                current_dir = "brake"
                pid_integral = 0.0
                pid_prev_error = None
            else:
                cx, cy, radius = result
                error = cx - center_x
                
                # Check if ball is close enough to brake
                if radius >= BALL_STOP_RADIUS_PX:
                    stopped_by_radius = True
                elif stopped_by_radius and radius < stop_hyst_px:
                    stopped_by_radius = False
                
                # If ball is centered or too close, brake
                if stopped_by_radius or abs(error) <= dead_zone_px:
                    target_rpm = 0
                    current_dir = "brake"
                    pid_integral = 0.0
                    pid_prev_error = None
                else:
                    # PID control
                    dt = (now - last_command_time) if last_command_time else 0.01
                    pid_integral += error * dt
                    pid_integral = max(-50, min(50, pid_integral))  # Clamp integral
                    
                    derivative = 0.0 if pid_prev_error is None else (error - pid_prev_error) / dt
                    u = (BALL_KP * error) + (BALL_KI * pid_integral) + (BALL_KD * derivative)
                    
                    pid_prev_error = error
                    
                    # Determine direction and RPM
                    positive_right = (u > 0)
                    current_dir = "cw" if positive_right else "ccw"
                    target_rpm = min(BALL_MAX_RPM, abs(u))
                
                last_command_time = now
                
                if now - last_print > 0.5:
                    last_print = now
                    print(f"[BALL] cx={cx:6.1f} err={error:+7.1f} r={radius:5.1f} "
                          f"dir={current_dir.upper()} rpm={target_rpm:.1f}")
            
            # Frame rate control
            elapsed = time() - loop_start
            frame_period = 1.0 / BALL_TRACKING_FPS
            if elapsed < frame_period:
                sleep(frame_period - elapsed)
    
    except Exception as e:
        print(f"❌ Ball tracking error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        ball_tracking_running = False
        target_rpm = 0
        current_dir = "brake"
        try:
            stream.stop()
        except:
            pass

# --- 6. Main Entry Point ---

def main():
    global target_rpm, current_dir, system_mode, current_duty_cycle, ball_tracking_running

    # Header
    print("\n" + "="*45)
    print("      REACTION WHEEL STABILIZATION SYSTEM")
    print("="*45)
    
    # 1. Selection Menu
    print("\nSelect Operating Mode:")
    print(" [1] Auto-Stabilize (IMU Active Correction)")
    print(" [2] Manual (Fixed RPM Commands)")
    print(" [3] Tennis Ball Tracking (Vision-based Control)")
    
    while True:
        choice = input("\nEnter choice (1, 2, or 3) > ").strip()
        if choice == "1":
            system_mode = "auto"
            print(">> INITIALIZING IN AUTO MODE...")
            break
        elif choice == "2":
            system_mode = "manual"
            print(">> INITIALIZING IN MANUAL MODE...")
            break
        elif choice == "3":
            system_mode = "ball_tracking"
            print(">> INITIALIZING IN BALL TRACKING MODE...")
            break
        else:
            print("Invalid selection. Please enter 1, 2, or 3.")


    # 2. Start Background Threads
    motor_thread = threading.Thread(target=motor_control_loop, daemon=True)
    motor_thread.start()

    stabilizer_thread = threading.Thread(target=maintain_loop, daemon=True)
    stabilizer_thread.start()

    # Start ball tracking if selected
    ball_tracker_thread = None
    if system_mode == "ball_tracking":
        ball_tracking_running = True
        ball_tracker_thread = threading.Thread(target=ball_tracking_loop, daemon=True)
        ball_tracker_thread.start()

    picam2 = start_camera_stream(port=CAMERA_STREAM_PORT)

    print("\n--- System Online ---")
    if system_mode == "ball_tracking":
        print("Ball Tracking Mode Active. Commands: 'stop', 'auto', 'manual', or 'exit'")
    else:
        print("Commands: 'auto', 'manual', 'brake', or '[speed] [dir]' (e.g., '50 cw')")

    try:
        while True:
            cmd_input = input("\nCommand > ").strip().lower()

            if not cmd_input:
                continue

            # Ball tracking mode specific commands
            if system_mode == "ball_tracking":
                if cmd_input == "stop":
                    ball_tracking_running = False
                    target_rpm = 0
                    current_dir = "brake"
                    print(">> Action: STOPPED BALL TRACKING")
                elif cmd_input == "auto":
                    ball_tracking_running = False
                    system_mode = "auto"
                    target_rpm = 0
                    current_dir = "brake"
                    print(">> Action: SWITCHED TO AUTO MODE")
                elif cmd_input == "manual":
                    ball_tracking_running = False
                    system_mode = "manual"
                    target_rpm = 0
                    current_dir = "brake"
                    print(">> Action: SWITCHED TO MANUAL MODE")
                elif cmd_input == "exit":
                    break
                else:
                    print("Ball Tracking Mode Commands: 'stop', 'auto', 'manual', or 'exit'")
            
            # Standard mode commands
            else:
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
                
                elif cmd_input == "ball":
                    system_mode = "ball_tracking"
                    ball_tracking_running = True
                    ball_tracker_thread = threading.Thread(target=ball_tracking_loop, daemon=True)
                    ball_tracker_thread.start()
                    print(">> Action: SWITCHED TO BALL TRACKING")
                    
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
                            print("Error: Unknown command. Use 'auto', 'manual', 'ball', or '[speed] [dir]'.")
                    except ValueError:
                        print("Error: Could not parse input.")

    except KeyboardInterrupt:
        print("\n\nUser interrupted. Shutting down...")
    finally:
        # Hardware Safety Shutdown
        ball_tracking_running = False
        picam2.stop_recording()
        rpwm.value = 0
        lpwm.value = 0
        r_en.off()
        l_en.off()
        print(">> GPIO cleaned up. Goodbye.")

if __name__ == "__main__":
    main()