#!/usr/bin/python
import time
import RPi.GPIO as GPIO
from Robot import Robot

robot = Robot()

robot.forward()
time.sleep(3)

robot.stop()
time.sleep(3)

robot.backward()
time.sleep(3)

robot.stop()
GPIO.cleanup()