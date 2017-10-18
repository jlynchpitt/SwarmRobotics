#!/usr/bin/python
import time
import Motor

rightMotor = Motor()

#clockwise
rightMotor.setMode(2)
rightMotor.setSpeed(50)

time.sleep(3)

#CounterClockwise
rightMotor.setMode(3)
rightMotor.setSpeed(25)

time.sleep(3)