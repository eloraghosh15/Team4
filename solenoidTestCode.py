import RPi.GPIO as GPIO
import time

# Configuration
# We use BCM numbering, so '24' means GPIO 24 (Physical Pin 18)
SOLENOID_PIN = 12

def setup():
    """Sets up the GPIO pins."""
    # Use Broadcom (BCM) pin numbering (matches GPIO labels like 'GPIO 24')
    GPIO.setmode(GPIO.BCM)
   
    # Set up the pin as an output
    GPIO.setup(SOLENOID_PIN, GPIO.OUT)
   
    # Ensure it starts OFF (Safety first!)
    GPIO.output(SOLENOID_PIN, GPIO.LOW)
    print(f"Setup complete. Controlling GPIO {SOLENOID_PIN} (Physical Pin 18)")

def loop():
    """Cycles the solenoid on and off."""
    try:
        print("Starting test cycle (Press CTRL+C to stop)...")
       
        for i in range(3):  # Run 3 times
            print(f"Cycle {i+1}: ON")
            GPIO.output(SOLENOID_PIN, GPIO.HIGH) # Turn Solenoid ON
            time.sleep(.2)
            

            print(f"Cycle {i+1}: OFF")
            GPIO.output(SOLENOID_PIN, GPIO.LOW)  # Turn Solenoid OFF
            time.sleep(5)                        # Wait 1 second
           
        print("Test complete.")

    except KeyboardInterrupt:
        print("\nTest stopped by user.")
        GPIO.output(SOLENOID_PIN, GPIO.LOW)  # Turn Solenoid OFF

       
    finally:
        # This block always runs, even if the code crashes or you stop it
        print("Cleaning up...")
        GPIO.output(SOLENOID_PIN, GPIO.LOW) # Turn OFF for safety
        GPIO.cleanup()                      # Reset pins to default state

if __name__ == "__main__":
    setup()
    loop()