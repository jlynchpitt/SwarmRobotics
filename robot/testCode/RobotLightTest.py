#!/usr/bin/python
import time
import RPi.GPIO as GPIO
from Robot import Robot
from smbus2 import SMBus

## address of the device from i2cdetect
sensorAddr =  0x44

## initialize smbus object using bus number
i2c = SMBus(1)

i2c.write_byte_data(sensorAddr, 1, 0xD)

robot = Robot()

robot.forward()

num = 50
temp = 0
while(temp < num):
	greenLow = i2c.read_byte_data(0x44, 9)
	greenHigh = i2c.read_byte_data(0x44,10)
	redLow = i2c.read_byte_data(0x44, 11)
	redHigh = i2c.read_byte_data(0x44, 12)
	blueLow = i2c.read_byte_data(0x44, 13)
	blueHigh = i2c.read_byte_data(0x44, 14)

	green = greenHigh<<8 | greenLow
	red = redHigh<<8 | redLow
	blue = blueHigh<<8 | blueLow

	print("Green: " +str(green) + " Red: " + str(red) + " Blue: " + str(blue))
	#print("Red: " +str(red))
	#print("Blue: " +str(blue))
	#print("\n")
	temp = temp +1
	time.sleep(0.5)

robot.stop()
GPIO.cleanup()
