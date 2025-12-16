#!/usr/bin/env python3
import serial
import time
import numpy as np
def sendString(port,baud,input,waitTime):
    ser=serial.Serial(port,baud)
    
    for x in input:
        ser.write(bytes(x,'utf-8'))
        # ser.write("%s"%(x).encode())
        time.sleep(waitTime)