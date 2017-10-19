#!/usr/bin/python
import time
import RPi.GPIO as GPIO
from Motor import Motor

#Init standby pin to high
GPIO.setmode(GPIO.BOARD)
GPIO.setup(15, GPIO.OUT)
GPIO.output(15, GPIO.HIGH)

leftMotor = Motor(29, 31, 33)
rightMotor = Motor(13, 11, 12)

#forward - right CW left CCW
rightMotor.setMode(2)
rightMotor.setSpeed(50)
leftMotor.setMode(3)
leftMotor.setSpeed(50)

time.sleep(3)

#backward - right CCW left CW
rightMotor.setMode(3)
rightMotor.setSpeed(25)
leftMotor.setMode(2)
leftMotor.setSpeed(25)

time.sleep(3)

GPIO.cleanup()