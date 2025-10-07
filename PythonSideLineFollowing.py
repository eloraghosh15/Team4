#!/usr/bin/env python3
import serial
import time
import numpy as np
from sendStringScript import sendString
leftMotor=int(100)
rightMotor=int(100)


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

            
                #Following is my control law, we're keeping it basic for now, writing good control law is your job
                #ok so high numbers(highest 7000) on the line follwing mean I am too far to the LEFT,
                #low numbers mean I am too far on the RIGHT, 3500 means I am at the middle
                #below is a basic control law you can send to your motors, with an exeption if z is a value greater than 7000, meaning the arduino code sees that the line sensor is on a cross. Feel free to take insperation from this,
            #but you will need to impliment a state machine similar to what you made in lab 2 (including a way of counting time without blocking)

                if not x < 15000: #LEFT SIDE 
                    # leftMotor=100+.02*x #now that we are SURE that z isnt the string cross, we cast z to an int and recalculate leftMotor and rightMotor, 
                    # rightMotor=250-.015*x
                    leftMotor=50+.02*x #now that we are SURE that z isnt the string cross, we cast z to an int and recalculate leftMotor and rightMotor, 
                    rightMotor=150-.01*x
                elif x < 11500:   #RIGHT SIDE
                    # leftMotor=250-.02*x #now that we are SURE that z isnt the string cross, we cast z to an int and recalculate leftMotor and rightMotor, 
                    # rightMotor=100+.028*x
                    leftMotor=150-.01*x #now that we are SURE that z isnt the string cross, we cast z to an int and recalculate leftMotor and rightMotor, 
                    rightMotor=50+.018*x

                elif y == 1:
                    leftMotor = 0
                    rightMotor = 0
                    # leftMotor=100+.01*x #now that we are SURE that z isnt the string cross, we cast z to an int and recalculate leftMotor and rightMotor, 
                    # rightMotor=250-.01*x
                    counter += 1
                    print("at intersection: "+str(counter))
                    if counter == 1:
                        leftMotor = 75
                        rightMotor = 75 
                        sendString('/dev/ttyACM0',115200,'<'+str(leftMotor)+','+str(rightMotor)+'>',0.0001)
                        while aiden:
                            if timer == 15:
                                aiden = False
                            sendString('/dev/ttyACM0',115200,'<'+str(leftMotor)+','+str(rightMotor)+'>',0.0001)
                            print("<"+ str(leftMotor)+","+str(rightMotor)+">")
                            print("hellooooooooo")
                            print(timer)
                            timer += 1
                        timer = 0
                        aiden = True
                        
                    elif counter == 2:

                        leftMotor = -300/2
                        rightMotor = 300/2
                        sendString('/dev/ttyACM0',115200,'<'+str(leftMotor)+','+str(rightMotor)+'>',0.0001)
                        while aiden:
                            if timer == 20:
                                aiden = False
                            print("Right motor "+str(leftMotor))
                            sendString('/dev/ttyACM0',115200,'<'+str(leftMotor)+','+str(rightMotor)+'>',0.0001)
                            print("<"+ str(leftMotor)+","+str(rightMotor)+">")
                            print("hellooooooooo")
                            print(timer)
                            timer += 1
                        timer = 0
                        aiden = True
                        leftMotor = 100
                        rightMotor = 100
                        while aiden:
                            if timer == 20:
                                aiden = False
                            print("Right motor "+str(leftMotor))
                            sendString('/dev/ttyACM0',115200,'<'+str(leftMotor)+','+str(rightMotor)+'>',0.0001)
                            print("<"+ str(leftMotor)+","+str(rightMotor)+">")
                            print("hellooooooooo")
                            print(timer)
                            timer += 1
                        timer = 0
                        aiden = True

                    elif counter == 3:
                        leftMotor = 300/2
                        rightMotor = -300/2
                        sendString('/dev/ttyACM0',115200,'<'+str(leftMotor)+','+str(rightMotor)+'>',0.0001)
                        while aiden:
                            if timer == 10:
                                aiden = False
                            print("Right motor "+str(leftMotor))
                            sendString('/dev/ttyACM0',115200,'<'+str(leftMotor)+','+str(rightMotor)+'>',0.0001)
                            print("<"+ str(leftMotor)+","+str(rightMotor)+">")
                            print("hellooooooooo")
                            print(timer)
                            timer += 1
                        timer = 0
                        aiden = True
                        leftMotor = 100
                        rightMotor = 100
                        while aiden:
                            if timer == 20:
                                aiden = False
                            print("Right motor "+str(leftMotor))
                            sendString('/dev/ttyACM0',115200,'<'+str(leftMotor)+','+str(rightMotor)+'>',0.0001)
                            print("<"+ str(leftMotor)+","+str(rightMotor)+">")
                            print("hellooooooooo")
                            print(timer)
                            timer += 1
                        timer = 0
                        aiden = True

                        leftMotor = 0
                        rightMotor = 0
                        while aiden:
                            if timer == 100:
                                aiden = False
                            print("Right motor "+str(leftMotor))
                            sendString('/dev/ttyACM0',115200,'<'+str(leftMotor)+','+str(rightMotor)+'>',0.0001)
                            print("<"+ str(leftMotor)+","+str(rightMotor)+">")
                            print("hellooooooooo")
                            print(timer)
                            timer += 1
                        timer = 0
                        aiden = True

                    elif counter > 3:
                        print("last one!!")


                        
                    


                                              
                    #do something here like incrimenting a value you call 'lines_hit' to one higher, and writing code to make sure that some time (1 second should do it) 
                    # passes between being able to incriment lines_hit so that it wont be incrimented a bunch of times when you hit your first cross. IE give your robot time to leave a cross
                    #before allowing lines_hit to be incrimented again.
                else:
                    leftMotor = 100
                    rightMotor = 100 
