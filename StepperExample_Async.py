#!/usr/bin/env python3
import serial
from time import sleep
import numpy as np
import RPi.GPIO as GPIO
import asyncio
import sys

#assign GPIO pins for motor
motor_channel = (13,16,19,20)

GPIO.setmode(GPIO.BCM)
GPIO.setup(motor_channel,GPIO.OUT)


async def StepperFullForward():
	print('clockwise full\n')
	GPIO.output(motor_channel, (GPIO.HIGH,GPIO.LOW,GPIO.LOW,GPIO.LOW))
	await asyncio.sleep(0.0001)
	GPIO.output(motor_channel, (GPIO.LOW,GPIO.HIGH,GPIO.LOW,GPIO.LOW))
	await asyncio.sleep(0.0001)
	GPIO.output(motor_channel, (GPIO.LOW,GPIO.LOW,GPIO.HIGH,GPIO.LOW))
	await asyncio.sleep(0.0001)
	GPIO.output(motor_channel, (GPIO.LOW,GPIO.LOW,GPIO.LOW,GPIO.HIGH))
	await asyncio.sleep(0.0001)

async def StepperFullReverse():
	print('counter clockwise full\n')
	GPIO.output(motor_channel, (GPIO.LOW,GPIO.LOW,GPIO.LOW,GPIO.HIGH))
	await asyncio.sleep(0.002)
	GPIO.output(motor_channel, (GPIO.LOW,GPIO.LOW,GPIO.HIGH,GPIO.LOW))
	await asyncio.sleep(0.002)
	GPIO.output(motor_channel, (GPIO.LOW,GPIO.HIGH,GPIO.LOW,GPIO.LOW))
	await asyncio.sleep(0.002)
	GPIO.output(motor_channel, (GPIO.HIGH,GPIO.LOW,GPIO.LOW,GPIO.LOW))
	await asyncio.sleep(0.002)

async def StepperHalfForward():
	print('clockwise half\n')
	GPIO.output(motor_channel, (GPIO.HIGH,GPIO.LOW,GPIO.LOW,GPIO.LOW))
	await asyncio.sleep(0.0001)
	GPIO.output(motor_channel, (GPIO.HIGH,GPIO.HIGH,GPIO.LOW,GPIO.LOW))
	await asyncio.sleep(0.0001)
	GPIO.output(motor_channel, (GPIO.LOW,GPIO.HIGH,GPIO.LOW,GPIO.LOW))
	await asyncio.sleep(0.0001)
	GPIO.output(motor_channel, (GPIO.LOW,GPIO.HIGH,GPIO.HIGH,GPIO.LOW))
	await asyncio.sleep(0.0001)
	GPIO.output(motor_channel, (GPIO.LOW,GPIO.LOW,GPIO.HIGH,GPIO.LOW))
	await asyncio.sleep(0.0001)
	GPIO.output(motor_channel, (GPIO.LOW,GPIO.LOW,GPIO.HIGH,GPIO.HIGH))
	await asyncio.sleep(0.0001)
	GPIO.output(motor_channel, (GPIO.LOW,GPIO.LOW,GPIO.LOW,GPIO.HIGH))
	await asyncio.sleep(0.0001)
	GPIO.output(motor_channel, (GPIO.HIGH,GPIO.LOW,GPIO.LOW,GPIO.HIGH))
	await asyncio.sleep(0.0001)

async def StepperHalfReverse():
	print('counter clockwise half\n')
	GPIO.output(motor_channel, (GPIO.LOW,GPIO.LOW,GPIO.LOW,GPIO.HIGH))
	await asyncio.sleep(0.002)
	GPIO.output(motor_channel, (GPIO.LOW,GPIO.LOW,GPIO.HIGH,GPIO.HIGH))
	await asyncio.sleep(0.002)
	GPIO.output(motor_channel, (GPIO.LOW,GPIO.LOW,GPIO.HIGH,GPIO.LOW))
	await asyncio.sleep(0.002)
	GPIO.output(motor_channel, (GPIO.LOW,GPIO.HIGH,GPIO.HIGH,GPIO.LOW))
	await asyncio.sleep(0.002)
	GPIO.output(motor_channel, (GPIO.LOW,GPIO.HIGH,GPIO.LOW,GPIO.LOW))
	await asyncio.sleep(0.0001)
	GPIO.output(motor_channel, (GPIO.HIGH,GPIO.HIGH,GPIO.LOW,GPIO.LOW))
	await asyncio.sleep(0.0001)
	GPIO.output(motor_channel, (GPIO.HIGH,GPIO.LOW,GPIO.LOW,GPIO.LOW))
	await asyncio.sleep(0.0001)
	GPIO.output(motor_channel, (GPIO.HIGH,GPIO.LOW,GPIO.LOW,GPIO.HIGH))
	await asyncio.sleep(0.0001)

async def main():
	motor_direction = input('Select motor direction: cf=clockwise (full) or ccf=counter clockwise (full) or ch = clockwise (half) or cch = counter clockwise (half)\n') # you add counterclockwise option

	while True:
		try:
			if(motor_direction == 'cf'):
				await StepperFullForward()
			elif(motor_direction == 'ccf'):
				await StepperFullReverse()
			elif(motor_direction =='ch'):
				await StepperHalfForward()
			elif(motor_direction =='cch'):
				await StepperHalfReverse()

		except KeyboardInterrupt as e: # have to put ctrl+c for this to stop
			motor_direction = input('q to quit\n') # you add counterclockwise option
			if(motor_direction == 'q'):
				print('Motor Stopped')

asyncio.run(main())
				

