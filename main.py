from gpiozero import PWMOutputDevice, DigitalOutputDevice, RotaryEncoder
from time import sleep, time
import threading
import sys

# --- 1. Pin Declarations (Using BCM GPIO Numbers) ---
# Note: Physical Pin 32 is GPIO 12, Physical Pin 33 is GPIO 13, etc.
RPWM_GPIO = 12  # Physical Pin 32
LPWM_GPIO = 13  # Physical Pin 33
R_EN_GPIO = 24  # Physical Pin 18
L_EN_GPIO = 25  # Physical Pin 22
ENC_A_GPIO = 22 # Physical Pin 15
ENC_B_GPIO = 23 # Physical Pin 16

# --- 2. Hardware Setup ---
rpwm = PWMOutputDevice(RPWM_GPIO, frequency = 1000)
lpwm = PWMOutputDevice(LPWM_GPIO, frequency = 1000)
r_en = DigitalOutputDevice(R_EN_GPIO)
l_en = DigitalOutputDevice(L_EN_GPIO)
encoder = RotaryEncoder(ENC_A_GPIO, ENC_B_GPIO, max_steps=0)

# --- 3. PD Constants & State ---
kp = 0.5  
kd = 0.2  
TICKS_PER_REV = 360 
MAX_ATTAINABLE_RPM = 100

# Global state variables shared between the input and the motor thread
target_rpm = 0
current_dir = "brake"
prev_error = 0
prev_time = time()
current_duty_cycle = 0

def get_velocity():
    """Calculates current angular velocity (RPM)."""
    global prev_time
    now = time()
    dt = now - prev_time
    
    if dt <= 0.02: # Prevent division by zero/jitter
        return 0, 0
    
    steps = encoder.steps
    encoder.steps = 0 # Reset for next window
    
    # RPM Calculation: (steps / ticks) / (seconds / 60)
    rpm = (steps / TICKS_PER_REV) / (dt / 60.0)
    prev_time = now
    return rpm, dt

def motor_control_loop():
    """
    This runs in the BACKGROUND. It keeps the motor spinning at the 
    target speed even while the user is typing commands.
    """
    global prev_error, current_duty_cycle, target_rpm, current_dir
    
    k = 0
    while True:
        # 1. Update Velocity
        current_rpm, dt = get_velocity()
        
        
        current_rpm = abs(current_rpm)

        # 2. Logic Cases
        if current_dir == "brake" or target_rpm == 0:
            rpwm.value = 0
            lpwm.value = 0
            current_duty_cycle = 0
        else:
            # Enable H-Bridge
            r_en.on()
            l_en.on()
            k = k+1
            if(k%40 == 0):
                print(f"RPS:  + {current_rpm}")
            # 3. PD Math (Your Logic)
            error = target_rpm - current_rpm
            derivative = (error - prev_error) / dt if dt > 0 else 0
            
            output = (kp * error) + (kd * derivative)
            
            # Apply adjustment to duty cycle (scaled for safety)
            current_duty_cycle += (output / 1000) 
            current_duty_cycle = max(0, min(1, current_duty_cycle))

            # Set Direction
            if current_dir == "cw":
                rpwm.value = current_duty_cycle
                lpwm.value = 0
            elif current_dir == "ccw":
                rpwm.value = 0
                lpwm.value = current_duty_cycle

            prev_error = error
        
        sleep(0.05) # Runs at 20Hz (every 50ms)

def main():
    global target_rpm, current_dir
    
    # Start the motor thread
    motor_thread = threading.Thread(target=motor_control_loop, daemon=True)
    motor_thread.start()

    print("--- Reaction Wheel Control ---")
    print("Commands: '<speed (rps)> <direction>' (e.g. '200 cw') or 'brake'")

    try:
        while True:
            cmd_input = input("\nCommand > ").strip().lower()

            if cmd_input == "brake":
                target_rpm = 0
                current_dir = "brake"
                print(">> Event: Braking motor.")
            else:
                try:
                    parts = cmd_input.split()
                    if len(parts) != 2:
                        print("Error: Use '<speed> <direction>'")
                        continue
                    
                    speed = float(parts[0])
                    direction = parts[1]

                    if 0 <= speed <= MAX_ATTAINABLE_RPM and direction in ["cw", "ccw"]:
                        target_rpm = speed
                        current_dir = direction
                        print(f">> New Target: {target_rpm} RPM {current_dir.upper()}")
                    else:
                        print(f"Safety: Max RPM is {MAX_ATTAINABLE_RPM}. Directions: cw/ccw.")
                
                except ValueError:
                    print("Error: Invalid speed number.")

    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        # Emergency Stop
        rpwm.value = 0
        lpwm.value = 0
        r_en.off()
        l_en.off()

if __name__ == "__main__":
    main()