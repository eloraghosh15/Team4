#!/usr/bin/env python3
import serial
import time
import numpy as np
from sendStringScript import sendString
import time


leftMotor=int(100)
rightMotor=int(100)

# PID constants
kpL = 1.0
kiL = 0.0
kdL = 0.1

# State variables
# State variables (initialize as needed)
cumErrorL = 0.0
lastSpeedErrorL = 0.0
priorTimeL = time.time()
maxErr = 1000.0
desVelL = 0.0  # Set this externally before calling

def drivePID(curr):
    global cumErrorL, lastSpeedErrorL, priorTimeL

    currentTime = time.time()
    elapsedTime = currentTime - priorTimeL
    if elapsedTime == 0:
        return 0.0

    error = desVelL - curr
    cumErrorL += error * elapsedTime

    # Integral windup prevention
    if cumErrorL > maxErr:
        cumErrorL = maxErr
    elif cumErrorL < -maxErr:
        cumErrorL = -maxErr

    rateError = (error - lastSpeedErrorL) / elapsedTime
    out = kpL * error + kiL * cumErrorL + kdL * rateError

    lastSpeedErrorL = error
    priorTimeL = currentTime

    return out

def findAngle(x):
    rightExtreme = 11500
    leftExtreme = 15000

    midPoint = (leftExtreme+rightExtreme)/2
    delta = leftExtreme-rightExtreme

    fractionDistance = (x-midPoint)/delta
    degree = fractionDistance*45

    return degree


if __name__ == '__main__':
    ser=serial.Serial('/dev/ttyACM0',115200)
    ser.reset_input_buffer() #clears anything the arduino has been sending while the Rpi isnt prepared to recieve.

    counter = 0
    aiden = True
    timer = 0

    while True:
        sendString('/dev/ttyACM0',115200,'<'+str(leftMotor)+','+str(rightMotor)+'>',0.0001)
        if ser.in_waiting > 0:  #we wait until the arduino has sent something to us before we try to read anything from the serial port.
                 
                line = ser.readline().decode('utf-8')
                line=line.split(',')
                #this splits the incoming string up by commas
                try:
                    
                    x=int(line[0])
                    y=int(line[1])
                    z=int(line[2]) #we dont convert this to a float becasue we went to be able to recieve the message that we are at a cross, which wont be an int. 
                    a = int(line[3])
                    # b = int(line[4])
                    # c= int(line[5])

                    print([x,y,z,a])
                except:
                    print("packet dropped") #this is designed to catch when python shoves bits on top of each other. 
                    z=0

                state = "FORWARD"
                #Following is my control law, we're keeping it basic for now, writing good control law is your job
                #ok so high numbers(highest 7000) on the line follwing mean I am too far to the LEFT,
                #low numbers mean I am too far on the RIGHT, 3500 means I am at the middle
                #below is a basic control law you can send to your motors, with an exeption if z is a value greater than 7000, meaning the arduino code sees that the line sensor is on a cross. Feel free to take insperation from this,
            #but you will need to impliment a state machine similar to what you made in lab 2 (including a way of counting time without blocking)

                        
                    


                                              
                    #do something here like incrimenting a value you call 'lines_hit' to one higher, and writing code to make sure that some time (1 second should do it) 
                    # passes between being able to incriment lines_hit so that it wont be incrimented a bunch of times when you hit your first cross. IE give your robot time to leave a cross
                    #before allowing lines_hit to be incrimented again.



                match state:
                    case "FORWARD":
                        angle = findAngle(x)
                        u = drivePID(angle)
                        leftMotor = 100-u
                        rightMotor = 100+u
                        # leftMotor=100
                        # rightMotor=100
                        # print("Right motor "+str(leftMotor))
                        sendString('/dev/ttyACM0',115200,'<'+str(leftMotor)+','+str(rightMotor)+'>',0.0001)
                        print("<"+ str(leftMotor)+","+str(rightMotor)+">")
                        # print("hellooooooooo")
                        
                    case "RIGHT":
                        print("right")

                    case "LEFT":
                        print("left")



