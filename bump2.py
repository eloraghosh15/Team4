#!/usr/bin/env python3
import serial
import time
import numpy as np

from sendStringScript import sendString
leftMotor=int(50)
rightMotor=int(50)

FL=int(1)
L=int(1)
ML=int(1) #a bump sensor that is unactivated starts at 1 (because they are pullups), hence why these are all one
MR=int(1)
R=int(1)
FR=int(1)
aiden = True
oscar = True 
state = "forward"
lastState = []
currentState = []

if __name__ == '__main__':
    ser=serial.Serial('/dev/ttyACM0',115200)
    #every time the serial port is opened, the arduino program will restart, very convient!
    ser.reset_input_buffer()
    ready = 0
    

    while aiden:
        
        #think of the below line as the default condition where no pairs of sensors are triggered as state 0, where the robot moves forward
        sendString('/dev/ttyACM0',115200,'<'+str(leftMotor)+','+str(rightMotor)+'>',0.0005)
        #ser.write(b'<'+bytes(str(leftMotor),'utf-8')+b','+bytes(str(rightMotor),'utf-8')+b'>')


        #why so I append '<' and '>' to the beginning and end of my message that I send to the arduino?

        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8')
                #ive just called 2 methods from the ser object, what do they do? read the documentation and find out!
            line=line.split(',')
                #this one i wont ask you about this one is pretty self explanitory

            try:
                    
                FR=int(line[0])
                R=int(line[1])
                MR=int(line[2])

                ML=int(line[3])  
                L=int(line[4])
                FL=int(line[5])

                print([FL,L,ML,MR,R,FR])
                lastState = [FL,L,ML,MR,R,FR]
                


                
            except:
                print("packetLost") 
                #why do I have this exepction? 



       
            #rudimentery state machine



        match state:
            case "head-on-collision":
                currentState = [FL,L,ML,MR,R,FR]
                # reset()
                sendString('/dev/ttyACM0',115200,'<'+str(-leftMotor)+','+str(-rightMotor)+'>',0.0005)
                time.sleep(1)
                sendString('/dev/ttyACM0',115200,'<'+str(-leftMotor)+','+str(rightMotor)+'>',0.0005)
                time.sleep(1)
                state = "forward"

            case "right-obstacle":
                # reset()
                currentState = [FL,L,ML,MR,R,FR]
                sendString('/dev/ttyACM0',115200,'<'+str(-leftMotor)+','+str(-rightMotor)+'>',0.0005)
                time.sleep(1)
                sendString('/dev/ttyACM0',115200,'<'+str(-leftMotor)+','+str(rightMotor)+'>',0.0005)
                time.sleep(1)
                state = "check"

            case "leftObstacle":
                # reset()
                currentState = [FL,L,ML,MR,R,FR]
                sendString('/dev/ttyACM0',115200,'<'+str(-leftMotor)+','+str(-rightMotor)+'>',0.0005)
                time.sleep(1)
                sendString('/dev/ttyACM0',115200,'<'+str(leftMotor)+','+str(-rightMotor)+'>',0.0005)
                time.sleep(1)
                state = "check"

            case "stop":
                sendString('/dev/ttyACM0',115200,'<'+str(0)+','+str(0)+'>',0.0005)
                time.sleep(2)
                sendString('/dev/ttyACM0',115200,'<'+str(0)+','+str(0)+'>',0.0005)
                time.sleep(1)
                aiden = False
                state = "check"
            
            case "forward":
                sendString('/dev/ttyACM0',115200,'<'+str(leftMotor)+','+str(rightMotor)+'>',0.0005)
                time.sleep(.4)
                state = "check"


            case "checkAfter":
                if FR < 1 and R <1 and MR < 1:
                    # print("stop")
                    sendString('/dev/ttyACM0',115200,'<'+str(0)+','+str(0)+'>',0.0005)
                    time.sleep(2)
                    sendString('/dev/ttyACM0',115200,'<'+str(0)+','+str(0)+'>',0.0005)
                    time.sleep(1)
                    aiden=False

                elif FL < 1 or L < 1:
                    # print("left")
                    state = "leftObstacle"
                elif ML < 1 or MR < 1:
                    # print("headon")
                    state = "head-on-collision"
                elif R < 1 or FR < 1:    
                    state = "right-obstacle"
                else:
                    state = "forward"

            case "check":
                if currentState == lastState:
                    while oscar:
                        currentState = [FL,L,ML,MR,R,FR]
                        if currentState == lastState:
                            pass
                        else:
                            oscar = False
                        state="checkAfter"
                else:
                    state = "checkAfter"
                
                    

