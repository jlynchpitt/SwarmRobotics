#!/usr/bin/python
import time
import RPi.GPIO as GPIO
from Robot import Robot
from smbus2 import SMBus

## address of the device from i2cdetect
sensorAddr =  0x48

## initialize smbus object using bus number
i2c = SMBus(1)

#i2c.write_byte_data(sensorAddr, 1, 0xD)

robot = Robot()

robot.forward()

num = 50
temp = 0
while(temp < num):
	tempWord = i2c.read_word_data(sensorAddr, 0)

	byte1 = tempWord & 0xFF
	byte2 = (tempWord & 0xFF00) >> 12

	temperature = ((byte1 << 4) + byte2)*0.0625

	print("Temperature: " + str(temperature))
	temp = temp +1
	time.sleep(0.5)

robot.stop()
GPIO.cleanup()
