#!/usr/bin/env python3
import serial
import time
import RPi.GPIO as GPIO
import math
import sys

# --- CONFIGURATION ---
TICKS_FOR_360 = 1900 #2300 
BASE_SPEED = 100
KP = 0.08  # Tune this: Start small
KD = 0.5   # Tune this: Damping term
TARGET_LINE_POS = 3500



# --- TURRET & STEPPER CONFIG ---
IN1, IN2, IN3, IN4 = 22, 23, 24, 25  # 
STEPPER_PINS = [IN1, IN3, IN2, IN4]
PIN_TURRET_IR = 14#14
PIN_SHOOTER = 12
STEP_DELAY = 0.01
FULL_STEP_SEQ = [(1,0,0,0), (1,1,0,0), (0,1,0,0), (0,1,1,0), (0,0,1,0), (0,0,1,1), (0,0,0,1), (1,0,0,1)]


class BattleBot:
    def __init__(self):
        self.setup_GPIO()
        self.ser = self.setup_serial()
        self.running = True
        self.wall_side = "RIGHT" # 
        self.last_line_error = 0
        self.seq_index = 0  # OSCAR ADDED   
    

    def setup_GPIO(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(STEPPER_PINS, GPIO.OUT)
        GPIO.setup(PIN_SHOOTER, GPIO.OUT)
        GPIO.output(PIN_SHOOTER, GPIO.LOW)
        GPIO.setup(PIN_TURRET_IR, GPIO.IN)
        self.step_index = 0 # Initialize stepper state


    def setup_serial(self):
        print("Connecting to Arduino...")
        ser = None
        
        # 1. Try both ports explicitly
        ports = ['/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyUSB0']
        for p in ports:
            try:
                print(f"Trying {p}...", end="")
                ser = serial.Serial(p, 115200, timeout=0.1)
                print(" OPEN!")
                break
            except:
                print(" Failed.")
        
        if not ser:
            print("CRITICAL ERROR: Could not find Arduino! Check USB cable.")
            sys.exit()
            
        # 2. FLUSH AND WAIT
        ser.reset_input_buffer()
        time.sleep(3) # Give Arduino time to reboot (Wait 3 full seconds)
        
        # 3. HANDSHAKE (Optional but recommended)
        # Send a "Ping" or just verify we can read anything
        print("Waiting for data stream...", end="")
        attempts = 0
        while attempts < 20:
            line = ser.readline().decode('utf-8').strip()
            if line: 
                print(f" Received: {line}")
                break
            time.sleep(0.1)
            attempts += 1
            
        print("Arduino Connected.")
        return ser


    def send(self, cmd):
        self.ser.write(f"<{cmd}>\n".encode('utf-8'))


    def read_data(self):
        if self.ser.in_waiting > 0:
            try: return self.ser.readline().decode('utf-8').strip()
            except: return None
        return None


    # --- VISUALIZATION HELPERS ---
    def polar_to_cartesian(self, angle, dist):
        rad = math.radians(angle)
        return (dist * math.cos(rad), dist * math.sin(rad))

    def save_visualization(self, scan_data, best_angle, best_dist):
        try:
            import matplotlib.pyplot as plt
            print("Saving map to 'scan_debug.png'...")
            
            # EDITED: Changed to 1 row, 2 columns to fit both plots
            fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(20, 10))
            
            # --- PLOT 1: SPATIAL MAP (Your Original Logic) ---
            ax1.plot(0, 0, 'ko', markersize=10, label="Robot")
            ax1.arrow(0, 0, 0, 20, head_width=5, fc='k', ec='k') 
            
            x_vals, y_vals = [], []
            dists_for_time_plot = [] # Collect dists for second plot
            
            for ang, dist in scan_data:
                x, y = self.polar_to_cartesian(ang, dist)
                x_vals.append(x)
                y_vals.append(y)
                dists_for_time_plot.append(dist) # Store for time plot
                
            ax1.scatter(x_vals, y_vals, c='blue', s=10, alpha=0.5, label="Scan Data")
            
            tx, ty = self.polar_to_cartesian(best_angle, best_dist)
            ax1.plot([0, tx], [0, ty], 'r-', linewidth=2, label="Selected Path")
            ax1.plot(tx, ty, 'rx', markersize=12, markeredgewidth=3)
            
            ax1.set_title(f"Spatial Map: {best_dist:.1f}cm @ {best_angle:.1f}°")
            ax1.set_aspect('equal')
            ax1.grid(True)
            ax1.legend()

            # --- PLOT 2: DISTANCE VS TIME (New Section) ---
            # Assuming scan_data is collected in chronological order, index = time
            ax2.plot(dists_for_time_plot, 'g-', linewidth=1, label="Distance Trace")
            
            # Draw a line for the detected minimum
            ax2.axhline(y=best_dist, color='r', linestyle='--', label=f"Min Detected ({best_dist:.1f})")
            
            ax2.set_title("Distance vs. Time (Sample Index)")
            ax2.set_xlabel("Sample Index (Time)")
            ax2.set_ylabel("Distance (cm)")
            ax2.grid(True)
            ax2.legend()
            
            plt.savefig('scan_debug.png')
            plt.close()
        except ImportError:
            print("Matplotlib not found. Skipping visualization.")
        except Exception as e:
            print(f"Vis Error: {e}")

    def step_motor(self, steps, direction=1):
        """ Moves stepper 'steps' times. Direction: 1=CW, -1=CCW """
        for _ in range(steps):
            self.step_index = (self.step_index + direction) % 8
            pattern = FULL_STEP_SEQ[self.step_index]
            GPIO.output(STEPPER_PINS, pattern)
            time.sleep(STEP_DELAY)

    def disable_stepper(self):
        GPIO.output(STEPPER_PINS, GPIO.LOW)

    def fire_solenoid(self):
        print("FIRING SOLENOID!")
        GPIO.output(PIN_SHOOTER, GPIO.HIGH) # Turn Solenoid ON
        time.sleep(.1)
        GPIO.output(PIN_SHOOTER, GPIO.LOW)

    # --- ROBUST SCANNING (From PIDMain.py) ---
    def find_robust_minimum(self, scan_data):
        if not scan_data: return 0, 9999
        scan_data.sort(key=lambda x: x[0])
        angles = [p[0] for p in scan_data]
        dists = [p[1] for p in scan_data]
        
        # Moving Average Window = 5
        min_dist = 9999
        best_angle = 0
        window = 5
        
        for i in range(len(dists)):
            start = max(0, i - window // 2)
            end = min(len(dists), i + window // 2 + 1)
            subset = dists[start:end]
            avg = sum(subset) / len(subset)
            
            if avg < min_dist:
                min_dist = avg
                best_angle = angles[i]
        return best_angle, min_dist

    # --- PHASE 0: LOCALIZE & APPROACH ---
    def phase_0_localization(self):
        print("PHASE 0: SCANNING...")
        self.ser.reset_input_buffer()
        self.send("S")  
        time.sleep(0.1) 
        self.send("R") 
        time.sleep(0.5) 

        self.send(f"M,{BASE_SPEED},{-BASE_SPEED}") 
        
        scan_data = []
        curr_ticks = 0
        start_time = time.time()

        while abs(curr_ticks) < TICKS_FOR_360:
            if time.time() - start_time > 8.0:
                self.send("M,0,0")
                break

            line = self.read_data()
            if line and line.startswith("D:"):
                try:
                    parts = line.split(":")[1].split(",")
                    dist = float(parts[0])
                    curr_ticks = max(abs(int(parts[1])), abs(int(parts[2])))
                    
                    if len(scan_data) % 15 == 0:
                        print(f"SCAN: {dist:.1f}cm | Ticks: {curr_ticks}")
                    
                    if 0 < dist < 300: 
                        angle = (curr_ticks / TICKS_FOR_360) * 360.0
                        scan_data.append((angle, dist))
                except: pass
            
        self.send("M,0,0") 
        time.sleep(0.5)    
        
        if len(scan_data) < 10:
            print("ERROR: No valid scan data.")
            return

        best_angle, best_dist = self.find_robust_minimum(scan_data)
        print(f"Closest Wall: {best_dist:.1f}cm @ {best_angle:.1f}°")

        heading_A = (best_angle - 90) % 360  
        heading_B = (best_angle + 90) % 360  

        def get_avg_dist(target_heading, data):
            total = 0
            count = 0
            for a, d in data:
                diff = abs(a - target_heading)
                if diff > 180: diff = 360 - diff
                
                if diff < 20: 
                    total += d
                    count += 1
            if count == 0: return 0
            return total / count

        avg_dist_A = get_avg_dist(heading_A, scan_data)
        avg_dist_B = get_avg_dist(heading_B, scan_data)

        print(f"Option A (Put Wall Right) sees avg: {avg_dist_A:.1f}cm")
        print(f"Option B (Put Wall Left)  sees avg: {avg_dist_B:.1f}cm")

        if avg_dist_A > avg_dist_B:
            choice = "A"
            target_heading = heading_A
        else:
            choice = "B"
            target_heading = heading_B

        if choice == "A" and avg_dist_A < 40 and avg_dist_B > 20:
             print("Override! Option A is blocked. Switching to B.")
             target_heading = heading_B
        elif choice == "B" and avg_dist_B < 40 and avg_dist_A > 20:
             print("Override! Option B is blocked. Switching to A.")
             target_heading = heading_A

        turn_needed = (target_heading + 180) % 360 - 180
        print(f"Executing Turn: {turn_needed:.1f}°")
        self.save_visualization(scan_data, best_angle, best_dist)

        turn_ticks = int((abs(turn_needed) / 360.0) * TICKS_FOR_360)
        print(f"Target Ticks: {turn_ticks}")

        self.send("R") 
        time.sleep(0.2)
        self.ser.reset_input_buffer() 

        if turn_needed > 0:
            self.send(f"M,{BASE_SPEED},{-BASE_SPEED}") 
        else:
            self.send(f"M,{-BASE_SPEED},{BASE_SPEED}") 
        
        curr_ticks = 0
        turn_start = time.time()
        
        while abs(curr_ticks) < turn_ticks:
            if time.time() - turn_start > 5.0: break
            
            line = self.read_data()
            if line and "D:" in line:
                 try: 
                     p = line.split(":")[1].split(",")
                     curr_ticks = max(abs(int(p[1])), abs(int(p[2])))
                     print(f"TURN: {curr_ticks} / {turn_ticks}")
                 except: pass

        self.send("M,0,0")
        time.sleep(0.5)

        print("Driving to Line...")
        self.send("M,150,150")
        self.ser.reset_input_buffer()
        
        while self.running:
            line = self.read_data()
            if line and line.startswith("D:"):
                try:
                    parts = line.split(":")[1].split(",")
                    if int(parts[3]) == 1:
                        print("LINE FOUND!")
                        self.send("M,0,0")
                        break
                except: pass
            time.sleep(0.01)

        self.send("M,0,0")

    # ---- PHASE 2: GET TO CENTER-------
    def phase_2_center(self):
        print(f"PHASE 2: GO TOWARDS CENTER(Wall was on {self.wall_side})")
        
        # --- STEP 1: EXECUTE BLIND TURN ONTO LINE ---
        # 90 degrees = approx 0.25 of a full rotation (2300 ticks)
        # Tune this multiplier (0.26, 0.24) if it over/undershoots!
        turn_ticks = int(0.25*.5 * TICKS_FOR_360) 

        distToBack = 150
        self.wall_side = "RIGHT" # 

        self.send("R") # Reset encoders
        time.sleep(0.1)

        if self.wall_side == "RIGHT":
            print("Turning LEFT towards center...")
            print(f'motoruoput{-BASE_SPEED},{BASE_SPEED}')
            self.send(f"M,{-BASE_SPEED},{BASE_SPEED}") # Left Turn
        else:
            print("Turning RIGHT towards center...")
            self.send(f"M,{BASE_SPEED},{-BASE_SPEED}") # Right Turn

        curr_ticks = 0
        turn_start = time.time()
        while abs(curr_ticks) < turn_ticks:
            if time.time() - turn_start > 3.0: break # Safety timeout
            line = self.read_data()
            if line and "D:" in line:
                 try: 
                     p = line.split(":")[1].split(",")
                     # Use max() to rely on the working encoder
                     curr_ticks = max(abs(int(p[1])), abs(int(p[2])))
                 except: pass
        
        self.send("M,0,0")
        time.sleep(0.5)


        self.send("S")  #s Arduino to Mode 1 (Ultrasonic)
        time.sleep(0.1) 
        
        self.ser.reset_input_buffer()

        self.send("R") 
        time.sleep(0.2)

        self.send(f"M,{BASE_SPEED},{BASE_SPEED}")
        distance = 999

        while distance > distToBack:
            line = self.read_data()
            
            if line and line.startswith("D:"):
                try:
                    parts = line.split(":")[1].split(",")
                    dist = float(parts[0])
                    distance = dist
                    print(f"Current Distance: {distance}")
                except:
                    pass
            time.sleep(0.05)

        self.send("M,0,0")
        


    # --- PHASE 2: LINE FOLLOW ---
    def phase_2_line_follow(self):
        print(f"PHASE 2: TURN & TRACK (Wall was on {self.wall_side})")
        

        turn_ticks = int(0.1 * TICKS_FOR_360) 
        self.send("L") 
        time.sleep(1)
        self.send("R") # Reset encoders
        time.sleep(0.1)
        self.wall_side = "RIGHT" # 
        self.ser.reset_input_buffer()
        time.sleep(0.2)



        if self.wall_side == "RIGHT":
            print("Turning LEFT towards center...")
            self.send(f"M,{BASE_SPEED},{-BASE_SPEED}") # Left Turn
        else:
            print("Turning RIGHT towards center...")
            self.send(f"M,{-BASE_SPEED},{BASE_SPEED}") # Right Turn
        time.sleep(1)
        # # Wait for turn to finish
        # curr_ticks = 0
        # turn_start = time.time()
        # while abs(curr_ticks) < turn_ticks:
        #     if time.time() - turn_start > 3.0: break # Safety timeout
        #     line = self.read_data()
        #     if line and "D:" in line:
        #          try: 
        #             p = line.split(":")[1].split(",")
        #              # Use max() to rely on the working encoder
        #             curr_ticks = max(abs(int(p[1])), abs(int(p[2])))
        #             # if len(scan_data) % 5 == 0: # Print every 10th reading to avoid spam
        #             print(f"RAW: {line} | Ticks: {p[1]}")

        #          except: pass
            
        self.send("M,0,0")
        time.sleep(0.5)


        self.send("L") 
        print("Starting PID Control...")
        
        while self.running:
            line = self.read_data()
            if line and line.startswith("L:"):
                try:
                    print(line)
                    parts = line.split(":")[1].split(",")
                    pos = int(parts[0])
                    cross = int(parts[1]) # The intersection flag
                    
                    # STOP CONDITION: Center Intersection Reached
                    if cross == 1:
                        print("CENTER REACHED! STOPPING.")
                        self.send("M,0,0")
                        return # Exit to Phase 3 (Shooting)

                    # PID CONTROL
                    if 0 < pos < 7000:
                        error = TARGET_LINE_POS - pos
                        derivative = error - self.last_line_error
                        correction = (KP * error) + (KD * derivative)
                        self.last_line_error = error
                        
                        left_motor = BASE_SPEED - correction
                        right_motor = BASE_SPEED + correction
                        
                        left_motor = max(0, min(200, int(left_motor)))
                        right_motor = max(0, min(200, int(right_motor)))
                        
                        self.send(f"M,{left_motor},{right_motor}")
                    


                except ValueError: pass
            time.sleep(0.005)

    def phase_3_turret_engage(self):
        print("PHASE 3: TURRET SCANNING...")

        TOTAL_STEPS_CW = 200    # CW from 0 -> +200
        TOTAL_STEPS_CCW = 400   # CCW from +200 -> -200

        scan_data = []
        nonDetected = True

        pos = 0  

        while nonDetected:
            # CW sweep: 0 -> +200
            for i in range(TOTAL_STEPS_CW):
                self.step_motor(1, direction=1)
                pos += 1

                ir_raw = GPIO.input(PIN_TURRET_IR)
                ir_val = 1 if ir_raw == 0 else 0
                scan_data.append(ir_val)

                if ir_raw == 0:   # hit
                    nonDetected = False
                    print("Hit during CW, stopping")
                    break

            if not nonDetected:
                break

            # CCW sweep: +200 -> -200
            for i in range(TOTAL_STEPS_CCW):
                self.step_motor(1, direction=-1)
                pos -= 1

                ir_raw = GPIO.input(PIN_TURRET_IR)
                ir_val = 1 if ir_raw == 0 else 0
                scan_data.append(ir_val)

                if ir_raw == 0:   # hit
                    nonDetected = False
                    print("Hit during CCW, stopping")
                    break

                if i % 50 == 0:
                    print(f"Scanning step {i}...")

            break  

        # Analyze the recorded data
        hits = [i for i, val in enumerate(scan_data) if val == 1]

        if not hits:
            print("SCAN FAILED: No IR Signal found.")
            self.disable_stepper()
            return

        print(f"Found {len(hits)} signal points.")

        target_index = int(sum(hits) / len(hits))
        print(f"Target Acquired at scan index: {target_index}")

        if target_index < TOTAL_STEPS_CW:
            # Target found in CW leg, position is + (target_index + 1) steps
            target_pos = target_index + 1
        else:
            # Target found in CCW leg
            idx_in_ccw = target_index - TOTAL_STEPS_CW  # 0..(TOTAL_STEPS_CCW-1)
            # At start of CCW, pos was +200 and decreased by idx_in_ccw+1
            target_pos = TOTAL_STEPS_CW - (idx_in_ccw + 1)

        print(f"Computed turret target_pos (steps from start): {target_pos}")
        print(f"Current turret pos before aiming: {pos}")

        delta_to_target = target_pos - pos
        if delta_to_target != 0:
            direction = 1 if delta_to_target > 0 else -1
            print(f"Moving {abs(delta_to_target)} steps to aim...")
            self.step_motor(abs(delta_to_target), direction=direction)
            pos = target_pos

        time.sleep(0.5)
        self.fire_solenoid()
        self.disable_stepper()
        time.sleep(5)
        
        if pos != 0:
            stepsToCenter = abs(pos)
            direction_center = -1 if pos > 0 else 1
            print(f"Rewinding {stepsToCenter} steps to center...")
            self.step_motor(stepsToCenter, direction=direction_center)
            pos = 0

    def irSensorTest(self):
        while True:
            self.disable_stepper()
            if GPIO.input(PIN_TURRET_IR) == 0:
                print("Turret pin is on")
            else:
                print("Turret pin is off")
        
    def run(self):
        try:
            self.phase_0_localization()
            time.sleep(1)
            self.phase_2_line_follow()
            # self.phase_2_center()
            time.sleep(2)
            # self.phase_3_turret_engage() 
            # self.irSensorTest()
        except KeyboardInterrupt:
            self.send("M,0,0")
            self.ser.reset_input_buffer()
            time.sleep(.05)
            self.send("R") 
            time.sleep(0.05)
            self.disable_stepper()


if __name__ == "__main__":
    bot = BattleBot()
    bot.run()