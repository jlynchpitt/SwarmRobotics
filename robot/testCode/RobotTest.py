#!/usr/bin/python
import time
import RPi.GPIO as GPIO
from Robot_Driver import Robot_Driver

robot = Robot_Driver()

robot.forward()
time.sleep(3)

robot.stop()
time.sleep(3)

robot.backward()
time.sleep(3)

robot.stop()
GPIO.cleanup()