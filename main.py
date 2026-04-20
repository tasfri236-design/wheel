import board
import busio
import math
import threading
import sys
from time import sleep, time
from gpiozero import PWMOutputDevice, DigitalOutputDevice, RotaryEncoder
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import BNO_REPORT_GYROSCOPE

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
    import board
    i2c = board.I2C()
    bno = BNO08X_I2C(i2c)
    bno.enable_feature(BNO_REPORT_GYROSCOPE)
    imu_connected = True
    print(">> BNO085 IMU Online (45° Tilt Compensation Active)")
except Exception as e:
    print(f">> IMU Error: {e}")
    imu_connected = False

# --- 4. Constants & State ---
kp = 0.8  
kd = 0.2  
TICKS_PER_REV = 360    
MAX_ATTAINABLE_RPM = 100
TILT_ANGLE = math.radians(45) # Pre-calculate radians

target_rpm = 0
current_dir = "brake"
prev_error = 0
prev_time = time()
current_duty_cycle = 0

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
    global prev_error, current_duty_cycle, target_rpm, current_dir
    
    k = 0
    while True:
        # 1. Update Velocity and time delta
        current_rpm, dt = get_velocity()
        current_rpm = abs(current_rpm)

        # 2. Case: Brake / Stop
        if current_dir == "brake" or target_rpm == 0:
            rpwm.value = 0
            lpwm.value = 0
            current_duty_cycle = 0
            prev_error = 0 
        
        # 3. Case: Spin (CW or CCW)
        else:
            r_en.on()
            l_en.on()

            # PD Math
            error = target_rpm - current_rpm
            derivative = (error - prev_error) / dt if dt > 0 else 0
            output = (kp * error) + (kd * derivative)
            
            # Adjustment logic
            current_duty_cycle += (output / 1000) 
            current_duty_cycle = max(0, min(1, current_duty_cycle))

            if current_dir == "cw":
                rpwm.value = current_duty_cycle
                lpwm.value = 0
            elif current_dir == "ccw":
                rpwm.value = 0
                lpwm.value = current_duty_cycle

            prev_error = error

        # --- Throttled Printing (Every 1 second) ---
        k += 1
        if k % 20 == 0 and current_dir != "brake":
            status = current_dir.upper() if target_rpm > 0 else "IDLE"
            
            # Calculate True Yaw from Tilted IMU
            true_yaw = 0.0
            if imu_connected:
                try:
                    gx, gy, gz = bno.gyro
                    # Tilted around X-axis: Combine Z and Y
                    true_yaw = (gz * math.cos(TILT_ANGLE)) - (gy * math.sin(TILT_ANGLE))
                except:
                    print("Hello")
                    pass # Skip if I2C read fails briefly

            print(f"[{status}] Motor: {current_rpm:6.1f} RPM | Duty: {current_duty_cycle:5.1%} | Platform Yaw: {true_yaw:6.3f} rad/s")
        
        sleep(0.05) 

def maintain():
    while True: 
        gx, gy, gz = bno.gyro
        # Tilted around X-axis: Combine Z and Y
        true_yaw = (gz * math.cos(TILT_ANGLE)) - (gy * math.sin(TILT_ANGLE))
        if(true_yaw>0):
            current_dir="brake"
            print("break")
            motor_control_loop()
            break
    return

def main():
    global target_rpm, current_dir
    
    motor_thread = threading.Thread(target=motor_control_loop, daemon=True)
    motor_thread.start()

    print("\n--- Reaction Wheel System Online ---")
    print("Commands: '100 cw', '50 ccw', 'brake'")

    try:
        while True:
            cmd_input = input("\nCommand > ").strip().lower()

            if cmd_input == "brake":
                target_rpm = 0
                current_dir = "brake"
                print(">> Action: BRAKING")
            else:
                try:
                    parts = cmd_input.split()
                    if len(parts) != 2:
                        continue
                    
                    speed = float(parts[0])
                    direction = parts[1]

                    if 0 <= speed <= MAX_ATTAINABLE_RPM and direction in ["cw", "ccw"]:
                        target_rpm = speed
                        current_dir = direction
                        current_duty_cycle = 0 # Fresh start for direction change
                    else:
                        print(f"Invalid input. Limit: {MAX_ATTAINABLE_RPM} RPM.")
                
                except ValueError:
                    print("Error: Speed must be a number.")

    except KeyboardInterrupt:
        print("\nShutting down safely...")
    finally:
        rpwm.value = 0
        lpwm.value = 0
        r_en.off()
        l_en.off()

if __name__ == "__main__":
    main()