#!/usr/bin/env python3
import serial
import time
import threading
import RPi.GPIO as GPIO
import numpy as np
import math
import sys

# --- MATPLOTLIB SAFETY CHECK ---
# This allows the code to run even if the installation failed
try:
    import matplotlib.pyplot as plt
    VISUALIZATION_ENABLED = True
except ImportError:
    print("WARNING: Matplotlib not installed. Visualization disabled.")
    print("Run: sudo apt-get install python3-matplotlib")
    VISUALIZATION_ENABLED = False

IN1, IN2, IN3, IN4 = 22, 23, 24, 25
STEPPER_PINS = [IN1, IN2, IN3, IN4]

PIN_TURRET_IR = 27
PIN_SHOOTER = 17

FULL_STEP_SEQ = [
    (1, 0, 1, 0),
    (0, 1, 1, 0),
    (0, 1, 0, 1),
    (1, 0, 0, 1),
]

# PID Const
Kp = 0.5 
Ki = 0.0
Kd = 0.1

class BattleBot:
    def __init__(self):
        self.setup_gpio()
        self.setup_serial()
        
        # State Variables
        self.running = True
        self.current_mode = "START"
        
        # Line Following Data
        self.last_error = 0
        self.integral = 0
        self.base_speed = 150
        
        # Ultrasonic Data
        self.distances = []
        self.scan_angles = []

        # Stepper State
        self.step_index = 0
        self.steps_per_rev = 200

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
                print("Check USB connection or Arduino code upload.")
                sys.exit()
        self.ser.reset_input_buffer()
        print("Serial Connected.")
        time.sleep(2) # Wait for Arduino reset

    # --- COMMUNICATION HELPERS ---
    def send_command(self, cmd):
        """ Sends string like <S> or <M,100,100> """
        msg = f"<{cmd}>\n"
        self.ser.write(msg.encode('utf-8'))

    def read_serial_data(self):
        """ Non-blocking read """
        if self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode('utf-8').strip()
                return line
            except:
                return None
        return None

    # --- STEPPER FUNCTIONS ---
    def step_motor(self, steps, direction=1):
        """ Blocking stepper move """
        for _ in range(steps):
            self.step_index = (self.step_index + direction) % 4
            pattern = FULL_STEP_SEQ[self.step_index]
            GPIO.output(STEPPER_PINS, pattern)
            time.sleep(0.01)

    def fire_solenoid(self):
        print("FIRING!")
        GPIO.output(PIN_SHOOTER, GPIO.HIGH)
        time.sleep(0.2)
        GPIO.output(PIN_SHOOTER, GPIO.LOW)

    # --- MATH & VISUALIZATION HELPERS ---
    def polar_to_cartesian(self, angle_deg, distance):
        """ Converts stepper angle and ultrasonic distance to X,Y map points """
        rad = math.radians(angle_deg)
        x = distance * math.cos(rad)
        y = distance * math.sin(rad)
        return (x, y)

    def fit_line_to_points(self, points):
        """ Fits a line (y = mx + c) to a set of X,Y points using basic least squares """
        if len(points) < 2:
            return None, None
        
        xs = [p[0] for p in points]
        ys = [p[1] for p in points]
        
        # Simple linear regression
        A = np.vstack([xs, np.ones(len(xs))]).T
        m, c = np.linalg.lstsq(A, ys, rcond=None)[0]
        return m, c
    
    def disable_stepper(self):
        """ NEW: Cuts power to stepper coils to prevent overheating """
        print("Disabling Stepper Power...")
        GPIO.output(STEPPER_PINS, GPIO.LOW)
    
    def save_visualization(self, raw_scan_data, identified_walls, furthest_wall_index):
        """ Generates a PNG of what the robot 'saw' """
        if not VISUALIZATION_ENABLED:
            return

        print("Saving Visualization to 'scan_debug.png'...")
        try:
            plt.figure(figsize=(10, 10))
            
            # 1. Plot Robot
            plt.plot(0, 0, 'ko', label="Robot", markersize=10) # Black Dot
            
            # 2. Plot Raw Points (Blue)
            raw_x = []
            raw_y = []
            for angle, dist in raw_scan_data:
                x, y = self.polar_to_cartesian(angle, dist)
                raw_x.append(x)
                raw_y.append(y)
            plt.scatter(raw_x, raw_y, c='blue', s=10, alpha=0.5, label="Raw Hits")
            
            # 3. Plot Identified Walls (Green Lines)
            for i, wall in enumerate(identified_walls):
                wx = [p[0] for p in wall]
                wy = [p[1] for p in wall]
                
                # Draw line fitting these points
                m, c = self.fit_line_to_points(wall)
                if m is not None:
                    # Calculate start and end X for the line to draw it nicely
                    min_x, max_x = min(wx), max(wx)
                    if min_x == max_x: continue # Avoid vertical line crash

                    line_x = np.linspace(min_x, max_x, 10)
                    line_y = m * line_x + c
                    
                    style = 'g-' # Green Line
                    width = 2
                    
                    if i == furthest_wall_index:
                        style = 'r-' # Highlight Furthest Wall in Red
                        width = 4
                        
                    plt.plot(line_x, line_y, style, linewidth=width)
                    
            plt.grid(True)
            plt.legend()
            plt.title(f"Lidar Scan Analysis\nIdentified {len(identified_walls)} Walls")
            plt.xlabel("X Distance (cm)")
            plt.ylabel("Y Distance (cm)")
            plt.axis('equal') # Ensure squares look like squares
            plt.savefig('scan_debug.png')
            plt.close()
            print("Saved 'scan_debug.png' successfully!")
        except Exception as e:
            print(f"Visualization Error: {e}")

    def find_walls_from_points(self, scan_data):
        """ 
        Implements the '3-Point' Wall Growing Algorithm.
        scan_data: list of (angle, distance)
        """
        cartesian_points = []
        for angle, dist in scan_data:
            if dist < 200: # Filter out infinity/max range
                cartesian_points.append(self.polar_to_cartesian(angle, dist))

        if len(cartesian_points) < 3:
            return []

        walls = []
        current_segment = [cartesian_points[0], cartesian_points[1]]
        
        # Iterate through points starting from the 3rd one
        for i in range(2, len(cartesian_points)):
            p3 = cartesian_points[i]
            
            # 1. Create line from first two points of the current segment
            p1 = current_segment[0]
            p2 = current_segment[-1] # Use the ends for better direction
            
            x1, y1 = p1
            x2, y2 = p2
            x0, y0 = p3
            
            A = y2 - y1
            B = x1 - x2
            C = x2*y1 - y2*x1
            
            denominator = math.sqrt(A*A + B*B)
            if denominator == 0: continue # p1 and p2 are same point

            distance = abs(A*x0 + B*y0 + C) / denominator
            
            # TOLERANCE CHECK (e.g., 5cm deviation allowed)
            if distance < 5.0:
                current_segment.append(p3)
            else:
                # Point is too far! Close this wall and start a new one.
                if len(current_segment) > 4: 
                    walls.append(current_segment)
                current_segment = [cartesian_points[i-1], p3] # Start new segment connecting prev point

        # Append last segment
        if len(current_segment) > 4:
            walls.append(current_segment)
            
        return walls

    # --- MAIN LOGIC PHASES ---

    def phase_1_ultrasonic_scan(self):
        print("PHASE 1: ULTRASONIC SCANNING (TURRET MODE)")
        
        # 1. Initialize
        self.send_command("S")  # Tell Arduino to start streaming Ultrasonic
        time.sleep(0.5)
        
        # 2. Perform Scan (0 to 270 degrees)
        # 200 steps = 360 deg -> 150 steps = 270 deg
        #  100 = 180 deg
        scan_data = []
        total_steps = 300 
        step_increment = 5 # Take a reading every 5 steps to save time

        wheelRadius = 2 #in
        robotRadius = 3.5 #in
        rCirc = 3.14*robotRadius**2
        wCirc = 3.14*wheelRadius**2

        rotationforTurn = rCirc/wCirc

        stepsDesired = rotationforTurn # *(steps/rotation for Dc motor)
        #now we  can do consistnet turns
        
        
        print("Scanning...")
        #
        for i in range(0, total_steps, step_increment):
            # rotate
            current_angle = (i / 400.0) * 360.0
            
            # Read Sensor (Clear buffer to get latest reading)
            self.ser.reset_input_buffer()
            # Give Arduino a tiny moment to push a new reading
            time.sleep(0.03) 
            
            line = self.read_serial_data()
            if line and line.startswith("U:"):
                try:
                    dist = float(line.split(":")[1])
                    if dist > 0:
                        scan_data.append((current_angle, dist))
                except:
                    pass

        # 3. Reset Turret to 0 (important!)
        print("Scan Complete. Resetting Turret...")
        self.step_motor(total_steps, direction=-1) # Reverse back to start
        self.disable_stepper()
        # 4. Analyze Data (The Hardened Localization Method)
        start_calc = time.time()
        walls = self.find_walls_from_points(scan_data)
        end_calc = time.time()
        
        print(f"Calculation Time: {(end_calc - start_calc)*1000:.2f} ms") # Print time in milliseconds
        print(f"Identified {len(walls)} distinct walls.")
        
        if not walls:
            print("Localization Failed: No walls found. Using Blind Drive.")
            self.save_visualization(scan_data, [], -1) # Save debug image even on failure
            # self.send_command("M,200,200")
            # time.sleep(1)
            # self.send_command("M,0,0")
            return

        # 5. Identify Furthest Wall
        furthest_wall_index = -1
        max_dist_to_wall = 0
        target_angle = 0
        
        for i, wall in enumerate(walls):
            # Find the center point of the wall
            avg_x = sum([p[0] for p in wall]) / len(wall)
            avg_y = sum([p[1] for p in wall]) / len(wall)
            
            # Distance from robot (0,0) to center of wall
            dist_to_wall = math.sqrt(avg_x**2 + avg_y**2)
            
            if dist_to_wall > max_dist_to_wall:
                max_dist_to_wall = dist_to_wall
                furthest_wall_index = i
                
                # Calculate Normal Angle to this wall
                m, c = self.fit_line_to_points(wall)
                if m is not None:
                    # Angle of the wall itself
                    wall_angle = math.degrees(math.atan(m))
                    # Angle perpendicular to wall = wall_angle + 90
                    target_angle = wall_angle + 90
        
        # --- NEW: SAVE THE DEBUG IMAGE ---
        self.save_visualization(scan_data, walls, furthest_wall_index)
        # ---------------------------------
        
        print(f"Furthest Wall Dist: {max_dist_to_wall:.1f}cm")
        print(f"Correction Angle Required: {target_angle:.1f} degrees")
        
        # 6. Align Chassis
        # Convert angle to motor spin duration (Need to calibrate this!)
        spin_dir = 1 if target_angle > 0 else -1
        # Example calibration: 90 degrees takes 1.0 second at speed 100
        spin_time = abs(target_angle) / 90.0 * 1.0 
        
        print(f"Aligning Chassis... ({spin_time:.2f}s)")
        # if spin_dir == 1:
        #     self.send_command("M,100,-100") # Right
        # else:
        #     self.send_command("M,-100,100") # Left
            
        # time.sleep(spin_time)
        # self.send_command("M,0,0")
        
        # # 7. Drive to Center
        # # Drive half the distance to the wall
        # drive_time = (max_dist_to_wall / 2) / 30.0 
        # print(f"Driving to center... ({drive_time:.2f}s)")
        # self.send_command("M,200,200")
        # time.sleep(drive_time)
        # self.send_command("M,0,0")

    def phase_2_line_follow(self):
        print("PHASE 2: LINE FOLLOWING")
        # Tell Arduino to switch to Line Mode
        # Note: We send M commands inside the loop, so Arduino handles parsing
        self.send_command("L")
        
        while self.running:
            line = self.read_serial_data()
            if line and line.startswith("L:"):
                # Parse L:3500,0
                parts = line.split(":")[1].split(",")
                position = int(parts[0])
                is_cross = int(parts[1])

                if is_cross == 1:
                    print("CROSS DETECTED - Switching to Shooting Phase")
                    self.send_command("M,0,0") # Stop
                    return # Exit this phase

                # PID Calculation
                error = 3500 - position
                P = error
                self.integral += error
                D = error - self.last_error
                self.last_error = error

                correction = (Kp * P) + (Ki * self.integral) + (Kd * D)
                
                # Apply to motors
                left_speed = self.base_speed - correction
                right_speed = self.base_speed + correction
                
                # Clamp speeds
                left_speed = max(0, min(400, left_speed))
                right_speed = max(0, min(400, right_speed))
                
                # Send command to Arduino
                self.send_command(f"M,{int(left_speed)},{int(right_speed)}")

    def phase_3_turret_engage(self):
        print("PHASE 3: TARGET ACQUISITION")
        
        target_found = False
        steps_scanned = 0
        
        # Sweep Stepper
        while steps_scanned < 200: # 1 full rev
            if GPIO.input(PIN_TURRET_IR) == 0: # Active Low usually
                print("TARGET LOCKED")
                target_found = True
                break
            
            self.step_motor(1, direction=1) # Step right
            steps_scanned += 1
            time.sleep(0.01)

        if target_found:
            self.fire_solenoid()
        else:
            print("No Target Found - Resetting")

    def run(self):
        try:
            # Sequence of Events
            self.phase_1_ultrasonic_scan()
            time.sleep(1)
            
            # self.phase_2_line_follow()
            # time.sleep(1)
            
            # self.phase_3_turret_engage()
            
            print("MISSION COMPLETE")

        except KeyboardInterrupt:
            print("Stopping...")
            self.send_command("M,0,0")
            GPIO.cleanup()

if __name__ == "__main__":
    bot = BattleBot()
    bot.run()