#!/usr/bin/env python3
import serial
import time
import threading  # <--- added
import numpy as np
from sendStringScript import sendString
import RPi.GPIO as GPIO  # assuming stepper/ultrasonic on Pi

# ----------------------------
# GPIO setup for stepper + ultrasonic
# ----------------------------
STEP_PIN = 17
DIR_PIN = 27
TRIG_PIN = 23
ECHO_PIN = 24

GPIO.setmode(GPIO.BCM)
GPIO.setup(STEP_PIN, GPIO.OUT)
GPIO.setup(DIR_PIN, GPIO.OUT)
GPIO.setup(TRIG_PIN, GPIO.OUT)
GPIO.setup(ECHO_PIN, GPIO.IN)

# ----------------------------
# Globals
# ----------------------------
leftMotor = int(100)
rightMotor = int(100)

kpL, kiL, kdL = 1.0, 0.0, 0.1
cumErrorL, lastSpeedErrorL, priorTimeL = 0.0, 0.0, time.time()
maxErr, desVelL = 1000.0, 0.0

# Stepper + ultrasonic globals
spinning = False
currentStep = 0
distance = 0.0
scan_data = []
stepsPerRev = 200
stepDelay = 0.002  # seconds per half-step (tune this)

# ----------------------------
# Helper Functions
# ----------------------------
def drivePID(curr):
    global cumErrorL, lastSpeedErrorL, priorTimeL
    currentTime = time.time()
    elapsedTime = currentTime - priorTimeL
    if elapsedTime == 0:
        return 0.0
    error = desVelL - curr
    cumErrorL += error * elapsedTime
    cumErrorL = max(-maxErr, min(maxErr, cumErrorL))
    rateError = (error - lastSpeedErrorL) / elapsedTime
    out = kpL * error + kiL * cumErrorL + kdL * rateError
    lastSpeedErrorL, priorTimeL = error, currentTime
    return out

def findAngle(x):
    rightExtreme = 11500
    leftExtreme = 15000
    midPoint = (leftExtreme + rightExtreme) / 2
    delta = leftExtreme - rightExtreme
    fractionDistance = (x - midPoint) / delta
    degree = fractionDistance * 45
    return degree

# ----------------------------
# THREAD FUNCTIONS
# ----------------------------
def spin_stepper():
    global currentStep, spinning
    GPIO.output(DIR_PIN, GPIO.HIGH)
    print("[Stepper] Spinning...")
    while spinning and currentStep < stepsPerRev:
        GPIO.output(STEP_PIN, GPIO.HIGH)
        time.sleep(stepDelay)
        GPIO.output(STEP_PIN, GPIO.LOW)
        time.sleep(stepDelay)
        currentStep += 1
    spinning = False
    print("[Stepper] Done spinning.")

def track_ultrasonic():
    global distance, spinning, currentStep, scan_data
    print("[Ultrasonic] Measuring distances...")
    while spinning:
        # Trigger
        GPIO.output(TRIG_PIN, True)
        time.sleep(0.00001)
        GPIO.output(TRIG_PIN, False)

        # Echo timing
        start_t = time.time()
        while GPIO.input(ECHO_PIN) == 0 and spinning:
            start_t = time.time()
        while GPIO.input(ECHO_PIN) == 1 and spinning:
            stop_t = time.time()

        elapsed = stop_t - start_t
        distance = (elapsed * 34300) / 2  # cm

        angle = (currentStep / stepsPerRev) * 360
        scan_data.append((angle, distance))
        print(f"[Scan] {angle:.1f}° -> {distance:.1f} cm")

        time.sleep(0.05)  # sampling rate ~20 Hz
    print("[Ultrasonic] Done scanning.")

# ----------------------------
# MAIN PROGRAM
# ----------------------------
if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyACM0', 115200)
    ser.reset_input_buffer()

    counter = 0
    aiden = True
    timer = 0
    laser = True
    spin = True
    IR = False
    map = []
    stepsperDegree = 360 / stepsPerRev
    stepPer10 = stepsperDegree * 10
    print(stepsperDegree)

    try:
        while True:
            sendString('/dev/ttyACM0', 115200, '<' + str(leftMotor) + ',' + str(rightMotor) + '>', 0.0001)

            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8')
                line = line.split(',')
                try:
                    x = int(line[0])
                    y = int(line[1])
                    z = int(line[2])
                    a = int(line[3])
                    print([x, y, z, a])
                except:
                    print("packet dropped")
                    z = 0

                state = "FORWARD"

                match state:
                    case "FORWARD":
                        angle = findAngle(x)
                        u = drivePID(angle)
                        leftMotor = 100 - u
                        rightMotor = 100 + u
                        sendString('/dev/ttyACM0', 115200, '<' + str(leftMotor) + ',' + str(rightMotor) + '>', 0.0001)
                        print("<" + str(leftMotor) + "," + str(rightMotor) + ">")

                    case "ULTRASPIN":
                        if not spinning:
                            print("Starting UltraSpin...")
                            spinning = True
                            currentStep = 0
                            scan_data = []

                            # Start concurrent threads
                            t_stepper = threading.Thread(target=spin_stepper, daemon=True)
                            t_sensor = threading.Thread(target=track_ultrasonic, daemon=True)
                            t_stepper.start()
                            t_sensor.start()

                    case _:
                        pass

            # Optional monitoring
            if spinning:
                print(f"[Live] Step={currentStep}, Distance={distance:.2f} cm")
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nStopping...")
        spinning = False
        GPIO.cleanup()
        ser.close()
