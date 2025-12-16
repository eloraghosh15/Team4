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
IN1, IN2, IN3, IN4 = 22, 23, 24, 25
STEPPER_PINS = [IN1, IN2, IN3, IN4]
PIN_TURRET_IR = 27
PIN_SHOOTER = 17

# --- ROBOT GEOMETRY CALIBRATION ---
# Tune these to make turns accurate!
WHEEL_RADIUS = 2.0      # Inches (approx)
ROBOT_RADIUS = 3.5      # Inches (approx)
SECONDS_PER_360 = 4.0 

STEP_DELAY = 0.01 
FULL_STEP_SEQ = [(1,0,1,0), (0,1,1,0), (0,1,0,1), (1,0,0,1)]

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

    def step_motor(self, steps, direction=1):
        for _ in range(steps):
            self.step_index = (self.step_index + direction) % 4
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
        Generates a PNG showing the robot, the raw points, and the identified target wall.
        raw_scan_data: List of (angle, distance) tuples
        """
        if not VISUALIZATION_ENABLED: return
        print("Saving Visualization to 'scan_debug.png'...")
        try:
            plt.figure(figsize=(10, 10))
            
            # 1. Plot Robot
            plt.plot(0, 0, 'ko', label="Robot Center", markersize=10)
            
            # 2. Plot Raw Sensor Hits
            raw_x = []
            raw_y = []
            for angle, dist in raw_scan_data:
                x, y = self.polar_to_cartesian(angle, dist)
                raw_x.append(x)
                raw_y.append(y)
            plt.scatter(raw_x, raw_y, c='blue', s=20, alpha=0.6, label="Lidar Hits")
            
            # 3. Highlight Closest Point (The "Wall Target")
            target_x, target_y = self.polar_to_cartesian(closest_wall_angle, closest_wall_dist)
            plt.plot([0, target_x], [0, target_y], 'r--', linewidth=2, label="Shortest Path to Wall")
            plt.scatter([target_x], [target_y], c='red', s=100, marker='X', label="Target Point")

            # 4. Draw Theoretical Parallel Line (Where the wall *should* be perpendicular)
            # Normal vector is (target_x, target_y).
            # Tangent vector (wall direction) is (-target_y, target_x)
            # Let's draw a small segment to visualize the wall plane
            wall_len = 20 # 20cm visual representation
            # Unit vector of wall direction
            mag = math.sqrt(target_x**2 + target_y**2)
            if mag > 0:
                ux = -target_y / mag
                uy = target_x / mag
                # Line segment endpoints
                p1x = target_x + ux * wall_len
                p1y = target_y + uy * wall_len
                p2x = target_x - ux * wall_len
                p2y = target_y - uy * wall_len
                plt.plot([p1x, p2x], [p1y, p2y], 'g-', linewidth=3, alpha=0.7, label="Inferred Wall Plane")

            plt.grid(True)
            plt.legend()
            plt.title(f"Wall Scan Analysis\nClosest Point: {closest_wall_dist:.1f}cm at {closest_wall_angle:.1f} deg")
            plt.xlabel("X Distance (cm)")
            plt.ylabel("Y Distance (cm)")
            plt.axis('equal')
            plt.savefig('scan_debug.png')
            plt.close()
            print("Saved 'scan_debug.png' successfully!")
        except Exception as e:
            print(f"Visualization Error: {e}")

    # --- LOGIC PHASES ---

    def phase_0_find_line(self):
        print("PHASE 0: STARTING SEQUENCE")
        self.send_command("S") 
        time.sleep(0.5)
        
        # 1. SCAN FOR CLOSEST WALL (Using Wheels)
        print("Scanning 360 with Chassis...")
        
        scan_data = [] 
        closest_wall_dist = 9999
        closest_wall_angle = 0
        
        # We will spin 360 degrees in small increments
        steps = 20
        step_time = SECONDS_PER_360 / steps
        self.send_command("M,100,-100") 

        for i in range(steps):
            # Spin
            # self.send_command("M,100,-100") 
            # time.sleep(step_time)
            # self.send_command("M,0,0") # Stop to measure
            # time.sleep(0.1)
            
            # Calculate Angle (0 to 360)
            current_angle = (i / steps) * 360.0
            
            # Read Sensor
            self.ser.reset_input_buffer()
            time.sleep(0.04)
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
                    
        self.send_command("M,0,0") # Stop to measure

        print(f"Closest Wall Found: {closest_wall_dist}cm at {closest_wall_angle} deg")

        # --- VISUALIZE RESULT ---
        self.save_visualization(scan_data, closest_wall_angle, closest_wall_dist)
        # ------------------------

        # 2. ALIGN PARALLEL
        # We want that wall to be at 90 degrees (Right side)
        # Turn needed = closest_wall_angle - 90
        # Since we spun 360, we are roughly back at 0.
        
        turn_angle = closest_wall_angle - 90
        print(f"Turning chassis {turn_angle} to align parallel.")
        
        spin_time = abs(turn_angle) / 360.0 * SECONDS_PER_360
        
        if turn_angle > 0:
            self.send_command("M,100,-100") # Turn Right
        else:
            self.send_command("M,-100,100") # Turn Left
            
        time.sleep(spin_time)
        self.send_command("M,0,0")
        
        # 3. ACTIVE WALL FOLLOWING (Until Line Found)
        print("Engaging Wall Follow Mode...")
        self.send_command("W") # Switch to Wall Follow Mode
        
        target_dist = 30.0 # Maintain 30cm from wall
        kp_wall = 3.0      # Steering sensitivity
        
        while self.running:
            line_data = self.read_serial_data()
            
            if line_data:
                # A. CHECK LINE FOUND
                if "L:FOUND" in line_data:
                    print("LINE INTERCEPTED!")
                    self.send_command("M,0,0")
                    break
                
                # B. CHECK DISTANCE & STEER
                if line_data.startswith("U:"):
                    try:
                        curr_dist = float(line_data.split(":")[1])
                        
                        # Bang-Bang or P-Control
                        error = target_dist - curr_dist
                        turn = int(error * kp_wall)
                        turn = max(-50, min(50, turn))
                        
                        # If error > 0 (Too Close): Turn Left (L slow, R fast)
                        # If error < 0 (Too Far): Turn Right (L fast, R slow)
                        # Note: Wall is on RIGHT.
                        
                        l_speed = 150 + turn
                        r_speed = 150 - turn
                        
                        self.send_command(f"M,{l_speed},{r_speed}")
                    except:
                        pass
            
            time.sleep(0.01)
            
        # # 4. TURN ONTO LINE
        # print("Turning 90 Left onto Line...")
        # turn_time_90 = SECONDS_PER_360 / 4.0
        # self.send_command("M,-100,100") 
        # time.sleep(turn_time_90)
        # self.send_command("M,0,0")

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

    def phase_3_turret_engage(self):
        print("PHASE 3: TARGET ACQUISITION")
        
        scan_data = []
        
        # 1. Scan 360
        for i in range(200):
            self.step_motor(1, direction=1)
            ir_val = 1 if GPIO.input(PIN_TURRET_IR) == 0 else 0
            scan_data.append(ir_val)
            time.sleep(0.005)

        # 2. Find Target
        hits = [i for i, val in enumerate(scan_data) if val == 1]
        
        if not hits:
            print("No Target Found.")
        else:
            target_index = int(sum(hits) / len(hits))
            rewind = 200 - target_index
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