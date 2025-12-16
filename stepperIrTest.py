import threading
import time
import RPi.GPIO as GPIO

# --- 1. CONFIGURATION ---

# MOTOR PINS (From your provided code)
IN1 = 22
IN2 = 23
IN3 = 24
IN4 = 25
MOTOR_PINS = (IN1, IN2, IN3, IN4)

# SENSOR & SHOOTER PINS (Re-assigned to avoid conflict)
PIN_TURRET_IR = 27  # Moved from 22
PIN_SHOOTER = 17    # Moved from 23

# --- STEPPER SEQUENCES (Bipolar/L298N) ---
# Full-step sequence (Strong torque)
FULL_STEP_SEQ = [
    (1, 0, 1, 0),
    (0, 1, 1, 0),
    (0, 1, 0, 1),
    (1, 0, 0, 1),
]

# MOVEMENT SETTINGS
STEP_DELAY = 0.008  # Adjust speed (Lower = Faster, too low = stall)
STEPS_PER_REV = 200 # Standard motor
STEPS_90_DEG = 50   # 90 degrees
SWEEP_STEPS = 100   # 180 degrees total sweep

class BattleBot:
    def __init__(self):
        self.setup_gpio()
        self.lock = threading.Lock()
        self.running = True
        
        # Stepper State Tracking
        self.seq_index = 0 # Keeps track of where we are in the (1,0,1,0) pattern
        
        # Shared Variables
        self.start_scan = False      
        self.scan_complete = False   
        self.target_found = False    
        
        # Start the Thread
        self.thread = threading.Thread(target=self.turret_logic)
        self.thread.daemon = True
        self.thread.start()

    def setup_gpio(self):
        GPIO.setmode(GPIO.BCM)
        # Setup Motor Pins
        GPIO.setup(MOTOR_PINS, GPIO.OUT)
        # Setup Peripherals
        GPIO.setup(PIN_SHOOTER, GPIO.OUT)
        GPIO.setup(PIN_TURRET_IR, GPIO.IN)

    # --- LOW LEVEL STEPPER FUNCTION ---
    def step_one_tick(self, direction):
        """
        Moves the motor exactly one 'tick' in the sequence.
        direction: 1 (Clockwise) or -1 (Counter-Clockwise)
        """
        # 1. Update the index based on direction
        # The % 4 wraps the index back to 0 if it goes past 3
        self.seq_index = (self.seq_index + direction) % 4
        
        # 2. Get the pattern (e.g., 1,0,1,0)
        pattern = FULL_STEP_SEQ[self.seq_index]
        
        # 3. Write to pins
        GPIO.output(MOTOR_PINS, pattern)
        
        # 4. Wait
        time.sleep(STEP_DELAY)

    # --- HELPER: ACTIVE SEARCH SECTOR ---
    def scan_sector(self, steps, direction):
        """
        Moves 'steps' ticks. Checks IR sensor between every tick.
        direction: 1 (CW) or -1 (CCW)
        """
        for _ in range(steps):
            
            # [A] CHECK SENSOR (High Priority)
            if GPIO.input(PIN_TURRET_IR) == 0: # 0 means DETECTED
                print(f"[Thread] TARGET LOCKED! Stopping.")
                
                with self.lock:
                    self.target_found = True
                    self.scan_complete = True
                
                # Stop Logic: Set all coils low to save power/hold
                # GPIO.output(MOTOR_PINS, (0,0,0,0)) 
                return True 

            # [B] STEP MOTOR (Low Priority)
            self.step_one_tick(direction)
            
        return False # Finished steps, nothing found

    # --- THE SENTRY LOGIC (Threaded) ---
    def turret_logic(self):
        while self.running:
            
            # 1. WAIT FOR COMMAND
            if not self.start_scan:
                time.sleep(0.1)
                continue

            print("[Thread] Sentry Mode Activated.")
            
            # 2. WIND UP (Move to Left Limit / -90)
            # We use direction -1 (CCW)
            print("[Thread] Moving to Left Limit...")
            if self.scan_sector(STEPS_90_DEG, -1):
                self.start_scan = False
                continue 

            # 3. THE INFINITE SEARCH LOOP
            while self.running:
                
                # A. SWEEP LEFT -> RIGHT (CW, +1)
                print("[Thread] Scanning Right...")
                if self.scan_sector(SWEEP_STEPS, 1):
                    break 

                # B. SWEEP RIGHT -> LEFT (CCW, -1)
                print("[Thread] Scanning Left...")
                if self.scan_sector(SWEEP_STEPS, -1):
                    break 
                
            self.start_scan = False

    # --- MAIN BRAIN ---
    def run(self):
        print("System Online. Starting Autonomous Loop.")
        
        try:
            while self.running:
                # --- STEP 1: NAVIGATION (Placeholder) ---
                print("Localizing")
                
                print("\n[Main] Navigating...")

                time.sleep(1) 
                
                # --- STEP 2: TRIGGER SCAN ---
                with self.lock:
                    self.scan_complete = False 
                    self.target_found = False
                
                self.start_scan = True 
                
                # --- STEP 3: WAIT FOR TURRET ---
                while not self.scan_complete:
                    time.sleep(0.1) 
                    
                # --- STEP 4: FIRE? ---
                if self.target_found:
                    print("[Main] FIRE!")
                    GPIO.output(PIN_SHOOTER, GPIO.HIGH)
                    time.sleep(0.5) 
                    GPIO.output(PIN_SHOOTER, GPIO.LOW)
                else:
                    print("[Main] Scan Failed.")

                time.sleep(1)

        except KeyboardInterrupt:
            print("Stopping...")
        finally:
            self.running = False
            GPIO.cleanup()

if __name__ == "__main__":
    bot = BattleBot()
    bot.run()