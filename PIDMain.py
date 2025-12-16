#!/usr/bin/env python3
import serial
import time
import RPi.GPIO as GPIO
import numpy as np
import math
import sys

# --- MATPLOTLIB ---
try:
    import matplotlib.pyplot as plt
    VISUALIZATION_ENABLED = True
except ImportError:
    VISUALIZATION_ENABLED = False

# --- CONFIGURATION ---
IN1, IN2, IN3, IN4 = 22, 23, 24, 25
STEPPER_PINS = [IN1, IN2, IN3, IN4]
PIN_TURRET_IR = 27
PIN_SHOOTER = 17

# --- CALIBRATION ---
TICKS_FOR_CHASSIS_360 = 1900 
print(f"CALIBRATION: Target Ticks for 360 turn = {TICKS_FOR_CHASSIS_360}")

STEP_DELAY = 0.01 
FULL_STEP_SEQ = [(1,0,0,0), (1,1,0,0), (0,1,0,0), (0,1,1,0), (0,0,1,0), (0,0,1,1), (0,0,0,1), (1,0,0,1)]

class BattleBot:
    def __init__(self):
        self.setup_gpio()
        self.setup_serial()
        self.running = True

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
                sys.exit()
        self.ser.reset_input_buffer()
        time.sleep(2) 

    def send_command(self, cmd):
        msg = f"<{cmd}>\n"
        self.ser.write(msg.encode('utf-8'))

    def read_serial_data(self):
        if self.ser.in_waiting > 0:
            try:
                return self.ser.readline().decode('utf-8').strip()
            except:
                return None
        return None

    def step_motor(self, steps, direction=1):
        pass 

    def disable_stepper(self):
        GPIO.output(STEPPER_PINS, GPIO.LOW)

    def fire_solenoid(self):
        GPIO.output(PIN_SHOOTER, GPIO.HIGH)
        time.sleep(0.2)
        GPIO.output(PIN_SHOOTER, GPIO.LOW)

    # --- VISUALIZATION & MATH ---
    def polar_to_cartesian(self, angle_deg, distance):
        rad = math.radians(angle_deg)
        x = distance * math.cos(rad)
        y = distance * math.sin(rad)
        return (x, y)

    def save_visualization(self, raw_scan_data, closest_wall_angle, closest_wall_dist, smooth_data=None):
        """ 
        Enhanced Visualization:
        1. Top-Down Map
        2. Distance vs Angle (With Raw + Smoothed Data)
        """
        if not VISUALIZATION_ENABLED: return
        print(f"Saving Visualization... ({len(raw_scan_data)} points)")
        try:
            fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8))
            
            # --- PLOT 1: MAP ---
            ax1.plot(0, 0, 'ko', label="Robot")
            ax1.arrow(0, 0, 0, 10, head_width=3, fc='k', ec='k')
            
            raw_x, raw_y = [], []
            angles, distances = [], []
            for angle, dist in raw_scan_data:
                x, y = self.polar_to_cartesian(angle, dist)
                raw_x.append(x); raw_y.append(y)
                angles.append(angle); distances.append(dist)
            
            if raw_x: ax1.scatter(raw_x, raw_y, c='blue', s=20, alpha=0.3, label="Raw Hits")
            
            target_x, target_y = self.polar_to_cartesian(closest_wall_angle, closest_wall_dist)
            ax1.plot([0, target_x], [0, target_y], 'r--', lw=2, label="Calculated Path")
            ax1.scatter([target_x], [target_y], c='red', s=150, marker='X', label="Target")

            # Turn Vector
            turn_angle = closest_wall_angle - 90
            ax1.text(0, -20, f"Turn: {turn_angle:.1f} deg", ha='center', color='red', weight='bold')
            ax1.grid(True); ax1.axis('equal'); ax1.legend()
            ax1.set_title("Robot Map (Top Down)")

            # --- PLOT 2: DISTANCE PROFILE ---
            if angles:
                # Plot Raw Data
                ax2.plot(angles, distances, 'b.', alpha=0.3, label="Raw Data")
                
                # Plot Smoothed Data (if available)
                if smooth_data:
                    sm_angles, sm_dists = zip(*smooth_data)
                    ax2.plot(sm_angles, sm_dists, 'k-', lw=2, label="Smoothed (Avg)")
                
                # Highlight the Min Point
                ax2.plot(closest_wall_angle, closest_wall_dist, 'rx', markersize=15, markeredgewidth=3, label="Selected Min")
                ax2.axvline(x=closest_wall_angle, color='r', linestyle='--', alpha=0.5)
            
            ax2.grid(True); ax2.legend()
            ax2.set_title("Lidar Profile: Distance vs Angle")
            ax2.set_xlabel("Angle (Deg)"); ax2.set_ylabel("Distance (cm)")
            
            plt.tight_layout()
            plt.savefig('scan_debug.png')
            plt.close()
        except Exception as e: print(f"Vis Error: {e}")

    # --- FILTERING ALGORITHM ---
    def find_robust_minimum(self, scan_data):
        """
        Applies a Moving Average filter to find the true 'center' of the closest wall,
        ignoring single-point noise spikes.
        """
        if not scan_data: return 0, 9999, []
        
        # Sort by angle first
        scan_data.sort(key=lambda x: x[0])
        
        angles = [p[0] for p in scan_data]
        dists = [p[1] for p in scan_data]
        
        # Window size for moving average (e.g., 5 points)
        window = 5
        smoothed = []
        
        min_dist = 9999
        best_angle = 0
        
        # Simple Moving Average
        for i in range(len(dists)):
            # Gather neighbors (handling boundaries somewhat simply)
            start = max(0, i - window // 2)
            end = min(len(dists), i + window // 2 + 1)
            subset = dists[start:end]
            
            avg_dist = sum(subset) / len(subset)
            smoothed.append((angles[i], avg_dist))
            
            if avg_dist < min_dist:
                min_dist = avg_dist
                best_angle = angles[i]
                
        return best_angle, min_dist, smoothed

    # --- PHASE 0 ---
    def phase_0_find_line(self):
        print("PHASE 0: ENCODER SCAN")
        self.send_command("R") 
        time.sleep(0.2) 
        self.ser.reset_input_buffer()
        
        # Spin
        self.send_command("M,120,-120") 
        
        scan_data = [] 
        current_ticks = 0
        
        # Collect Data
        while abs(current_ticks) < TICKS_FOR_CHASSIS_360:
            line = self.read_serial_data()
            if line and line.startswith("D:"):
                try:
                    parts = line.split(":")[1].split(",")
                    dist = float(parts[0])
                    current_ticks = int(parts[1]) 
                    angle = (abs(current_ticks) / TICKS_FOR_CHASSIS_360) * 360.0
                    
                    if dist > 0 and dist < 200: # Ignore infinity
                        scan_data.append((angle, dist))
                except: pass
            time.sleep(0.001) 
            
        self.send_command("M,0,0") 
        print("SPIN TRACK DONE")
        time.sleep(3)
        # --- ANALYSIS ---
        # Use the new robust finder
        best_angle, best_dist, smooth_data = self.find_robust_minimum(scan_data)
        
        print(f"Raw Min: {min([d for a,d in scan_data]) if scan_data else 0:.1f}")
        print(f"Robust Min: {best_dist:.1f}cm at {best_angle:.1f} deg")
        
        self.save_visualization(scan_data, best_angle, best_dist, smooth_data)

        # ALIGN PARALLEL
        turn_angle = best_angle - 90
        turn_ticks = int((abs(turn_angle) / 360.0) * TICKS_FOR_CHASSIS_360)
        
        print(f"Turning {turn_angle:.1f} deg ({turn_ticks} ticks)")
        self.send_command("R")
        time.sleep(0.2)
        
        if turn_angle > 0: self.send_command("M,150,-150")
        else: self.send_command("M,-150,150")
            
        current_ticks = 0
        while abs(current_ticks) < turn_ticks:
            line = self.read_serial_data()
            if line and line.startswith("D:"):
                try: current_ticks = int(line.split(":")[1].split(",")[1])
                except: pass
            time.sleep(0.001)
        
        self.send_command("M,0,0")
        
        # DRIVE UNTIL LINE
        print("Driving to Line...")
        self.send_command("M,150,150")
        
        while self.running:
            line = self.read_serial_data()
            if line and line.startswith("D:"):
                parts = line.split(":")[1].split(",")
                if int(parts[3]) == 1:
                    print("LINE FOUND!")
                    self.send_command("M,0,0")
                    break
            time.sleep(0.01)

    def run(self):
        try:
            self.phase_0_find_line()
            print("DONE")
        except KeyboardInterrupt:
            self.send_command("M,0,0")
            GPIO.cleanup()

if __name__ == "__main__":
    bot = BattleBot()
    bot.run()