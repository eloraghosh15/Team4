#!/usr/bin/env python3
import serial
import time
import threading
import RPi.GPIO as GPIO
import numpy as np
import math
import sys

# --- MATPLOTLIB SAFETY CHECK ---
try:
    import matplotlib.pyplot as plt
    VISUALIZATION_ENABLED = True
except ImportError:
    print("WARNING: Matplotlib not installed. Visualization disabled.")
    VISUALIZATION_ENABLED = False

# --- CONFIGURATION ---
# Stepper Pins (ONLY for Turret/Shooter)
IN1, IN2, IN3, IN4 = 22, 23, 24, 25
STEPPER_PINS = [IN1, IN2, IN3, IN4]
PIN_TURRET_IR = 27
PIN_SHOOTER = 17

# --- ROBOT GEOMETRY CALIBRATION ---
# Tune these to make turns accurate!
SECONDS_PER_360 = 4.0   # Time to spin 360 deg using DC motors at speed 150

# --- STEPPER TUNING (For Turret Only) ---
STEP_DELAY = 0.01 
FULL_STEP_SEQ = [
    (1, 0, 0, 0), (1, 1, 0, 0), (0, 1, 0, 0), (0, 1, 1, 0),
    (0, 0, 1, 0), (0, 0, 1, 1), (0, 0, 0, 1), (1, 0, 0, 1)
]

# PID Constants
Kp = 0.5 
Ki = 0.0
Kd = 0.1

class BattleBot:
    def __init__(self):
        self.setup_gpio()
        self.setup_serial()
        self.running = True
        self.base_speed = 150
        self.step_index = 0

    def setup_gpio(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(STEPPER_PINS, GPIO.OUT)
        GPIO.setup(PIN_SHOOTER, GPIO.OUT)
        GPIO.setup(PIN_TURRET_IR, GPIO.IN)

    def setup_serial(self):
        try:
            self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)
        except:
            try:
                self.ser = serial.Serial('/dev/ttyACM1', 115200, timeout=0.1)
            except:
                print("Error: Could not find Arduino A-Star")
                sys.exit()
        self.ser.reset_input_buffer()
        print("Serial Connected.")
        time.sleep(2) 

    # --- HELPERS ---
    def send_command(self, cmd):
        msg = f"<{cmd}>\n"
        self.ser.write(msg.encode('utf-8'))

    def read_serial_data(self):
        if self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode('utf-8').strip()
                return line
            except:
                return None
        return None

    # --- STEPPER FUNCTIONS (FOR TURRET/SHOOTER ONLY) ---
    def step_motor(self, steps, direction=1):
        for _ in range(steps):
            self.step_index = (self.step_index + direction) % 8
            pattern = FULL_STEP_SEQ[self.step_index]
            GPIO.output(STEPPER_PINS, pattern)
            time.sleep(STEP_DELAY)

    def disable_stepper(self):
        GPIO.output(STEPPER_PINS, GPIO.LOW)

    def fire_solenoid(self):
        print("FIRING!")
        GPIO.output(PIN_SHOOTER, GPIO.HIGH)
        time.sleep(0.2)
        GPIO.output(PIN_SHOOTER, GPIO.LOW)

    # --- VISUALIZATION METHOD ---
    def polar_to_cartesian(self, angle_deg, distance):
        rad = math.radians(angle_deg)
        x = distance * math.cos(rad)
        y = distance * math.sin(rad)
        return (x, y)

    def save_visualization(self, raw_scan_data, closest_wall_angle, closest_wall_dist):
        """ 
        Generates a 2-panel PNG: Top-down Map and Distance vs Angle plot.
        """
        if not VISUALIZATION_ENABLED: return
        print("Saving Visualization to 'scan_debug.png'...")
        try:
            fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8))
            
            # PLOT 1: Top-Down Cartesian Map
            ax1.plot(0, 0, 'ko', label="Robot Center", markersize=12)
            ax1.arrow(0, 0, 0, 10, head_width=3, head_length=3, fc='k', ec='k')
            
            raw_x = []
            raw_y = []
            angles = []
            distances = []
            
            for angle, dist in raw_scan_data:
                x, y = self.polar_to_cartesian(angle, dist)
                raw_x.append(x)
                raw_y.append(y)
                angles.append(angle)
                distances.append(dist)
            
            if raw_x:
                ax1.scatter(raw_x, raw_y, c='blue', s=20, alpha=0.6, label="All Lidar Hits")
            
            target_x, target_y = self.polar_to_cartesian(closest_wall_angle, closest_wall_dist)
            ax1.plot([0, target_x], [0, target_y], 'r--', linewidth=2, label="Shortest Path")
            ax1.scatter([target_x], [target_y], c='red', s=100, marker='X', label="Target Point")

            wall_len = 30 
            mag = math.sqrt(target_x**2 + target_y**2)
            if mag > 0:
                ux = -target_y / mag
                uy = target_x / mag
                p1x = target_x + ux * wall_len
                p1y = target_y + uy * wall_len
                p2x = target_x - ux * wall_len
                p2y = target_y - uy * wall_len
                ax1.plot([p1x, p2x], [p1y, p2y], 'g-', linewidth=3, alpha=0.7, label="Inferred Wall")

            ax1.grid(True)
            ax1.legend()
            ax1.set_title(f"Top-Down Map\nClosest: {closest_wall_dist:.1f}cm at {closest_wall_angle:.1f} deg")
            ax1.axis('equal')

            # PLOT 2: Distance vs Angle
            sorted_data = sorted(zip(angles, distances))
            if sorted_data:
                s_angles = [a for a, d in sorted_data]
                s_dists = [d for a, d in sorted_data]
                ax2.plot(s_angles, s_dists, 'bo-', markersize=4, linewidth=1, label="Measured Distances")
                ax2.plot(closest_wall_angle, closest_wall_dist, 'rx', markersize=12, markeredgewidth=3, label="Min Point")
                ax2.axhline(y=closest_wall_dist, color='r', linestyle='--', alpha=0.3)
            
            ax2.grid(True)
            ax2.legend()
            ax2.set_title("Scan Profile: Distance vs Angle")
            ax2.set_xlabel("Angle (Degrees)")
            ax2.set_ylabel("Distance (cm)")

            plt.tight_layout()
            plt.savefig('scan_debug.png')
            plt.close()
            print("Saved 'scan_debug.png' successfully!")
        except Exception as e:
            print(f"Visualization Error: {e}")

    # --- LOGIC PHASES ---

    def phase_0_find_line(self):
        print("PHASE 0: STARTING SEQUENCE (Chassis Scan)")
        self.send_command("S") 
        time.sleep(0.5)
        
        # 1. SCAN FOR CLOSEST WALL (Using DC Motors)
        print("Scanning 360 with Chassis...")
        
        scan_data = [] 
        closest_wall_dist = 9999
        closest_wall_angle = 0
        
        # START SPINNING (DC Motors)
        # Using a Continuous Spin logic to prevent stutter
        self.send_command("M,100,-100") 
        start_spin_time = time.time()
        
        # Spin for exactly 4 seconds (or calibrated time)
        while (time.time() - start_spin_time) < SECONDS_PER_360:
            elapsed = time.time() - start_spin_time
            current_angle = (elapsed / SECONDS_PER_360) * 360.0
            
            # Read Sensor
            line = self.read_serial_data()
            if line and line.startswith("U:"):
                try:
                    dist = float(line.split(":")[1])
                    if dist > 0:
                        scan_data.append((current_angle, dist))
                        if dist < closest_wall_dist:
                            closest_wall_dist = dist
                            closest_wall_angle = current_angle
                except:
                    pass
            
            # No sleep here needed, we want to catch as many packets as possible
            # Arduino sends every 60ms, Python loops faster
            # Just read as fast as we can
        
        # STOP SPINNING (DC Motors)
        self.send_command("M,0,0") 
        time.sleep(0.5) # Allow inertia to settle

        print(f"Closest Wall Found: {closest_wall_dist}cm at {closest_wall_angle:.1f} deg")

        # --- VISUALIZE RESULT ---
        self.save_visualization(scan_data, closest_wall_angle, closest_wall_dist)
        # ------------------------

        # 2. ALIGN PARALLEL (Using DC Motors)
        turn_angle = closest_wall_angle - 90
        print(f"Turning chassis {turn_angle:.1f} deg to align parallel.")
        
        spin_time = abs(turn_angle) / 360.0 * SECONDS_PER_360
        
        if turn_angle > 0:
            self.send_command("M,150,-150") # Turn Right
        else:
            self.send_command("M,-150,150") # Turn Left
            
        time.sleep(spin_time)
        self.send_command("M,0,0")
        
        # 3. ACTIVE WALL FOLLOWING
        print("Engaging Wall Follow Mode...")
        self.send_command("W") # Switch to Wall Follow Mode
        
        target_dist = 30.0 # Maintain 30cm from wall
        kp_wall = 3.0      # Steering sensitivity
        
        while self.running:
            line_data = self.read_serial_data()
            if line_data:
                if "L:FOUND" in line_data:
                    print("LINE INTERCEPTED!")
                    self.send_command("M,0,0")
                    break
                
                if line_data.startswith("U:"):
                    try:
                        curr_dist = float(line_data.split(":")[1])
                        error = target_dist - curr_dist
                        turn = int(error * kp_wall)
                        turn = max(-50, min(50, turn))
                        
                        l_speed = 150 + turn
                        r_speed = 150 - turn
                        
                        self.send_command(f"M,{l_speed},{r_speed}")
                    except:
                        pass
            time.sleep(0.01)
            
        # 4. TURN ONTO LINE
        print("Turning 90 Left onto Line...")
        turn_time_90 = SECONDS_PER_360 / 4.0
        self.send_command("M,-150,150") 
        time.sleep(turn_time_90)
        self.send_command("M,0,0")

    def phase_2_line_follow(self):
        print("PHASE 2: LINE FOLLOWING")
        self.send_command("L")
        self.last_error = 0
        
        while self.running:
            line = self.read_serial_data()
            if line and line.startswith("L:"):
                parts = line.split(":")[1].split(",")
                if len(parts) < 2: continue 
                
                position = int(parts[0])
                is_cross = int(parts[1])

                if is_cross == 1:
                    print("CROSS DETECTED - Stopping at Center")
                    self.send_command("M,0,0") 
                    return 

                # PID Control
                error = 3500 - position
                P = error
                D = error - self.last_error
                self.last_error = error
                correction = (Kp * P) + (Kd * D)
                
                left_speed = self.base_speed - correction
                right_speed = self.base_speed + correction
                
                left_speed = max(0, min(400, left_speed))
                right_speed = max(0, min(400, right_speed))
                
                self.send_command(f"M,{int(left_speed)},{int(right_speed)}")

    # Phase 3 uses STEPPER for the TURRET aiming
    def phase_3_turret_engage(self):
        print("PHASE 3: TARGET ACQUISITION")
        scan_data = []
        
        # 1. Scan 360 using STEPPER
        for i in range(400): # 400 steps for 360 deg in half-step
            self.step_motor(1, direction=1)
            ir_val = 1 if GPIO.input(PIN_TURRET_IR) == 0 else 0
            scan_data.append(ir_val)
            time.sleep(0.005)

        hits = [i for i, val in enumerate(scan_data) if val == 1]
        
        if not hits:
            print("No Target Found.")
        else:
            target_index = int(sum(hits) / len(hits))
            rewind = 400 - target_index
            print(f"Target at {target_index}. Rewinding {rewind}...")
            self.step_motor(rewind, direction=-1)
            
            time.sleep(0.5)
            self.fire_solenoid()
        
        self.disable_stepper()

    def run(self):
        try:
            self.phase_0_find_line()
            time.sleep(1)
            # self.phase_2_line_follow()
            # time.sleep(1)
            # self.phase_3_turret_engage()
            print("MISSION COMPLETE")
        except KeyboardInterrupt:
            print("Stopping...")
            self.send_command("M,0,0")
            self.disable_stepper()
            GPIO.cleanup()

if __name__ == "__main__":
    bot = BattleBot()
    bot.run()