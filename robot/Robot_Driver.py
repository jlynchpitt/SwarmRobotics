#!/usr/bin/python

########################################################
# Class function:
#	Driver for sending commands to the motor driver/motors
########################################################

import RPi.GPIO as GPIO
from Motor import Motor

#Mode = 0: 
#Mode = 1: stop
#Mode = 2: clockwise
#Mode = 3: counterclockwise
class Robot_Driver:
	CLOCKWISE = 2
	CCLOCKWISE = 3

	def __init__(self):
		#Init standby pin to high
		GPIO.setmode(GPIO.BOARD)
		GPIO.setup(15, GPIO.OUT)
		GPIO.output(15, GPIO.HIGH)

		self.leftMotor = Motor(29, 31, 33)
		self.rightMotor = Motor(13, 11, 12)
		
	def rightWheel(self, speed):
		#clockwise forward
		if(speed >= 0):
			self.rightMotor.setMode(CLOCKWISE)
			self.rightMotor.setSpeed(speed)
		else:
			self.rightMotor.setMode(CCLOCKWISE)
			self.rightMotor.setSpeed(-1*speed)
			
	def leftWheel(self, speed):
		#counter clockwise forward
		if(speed >= 0):
			self.leftMotor.setMode(CCLOCKWISE)
			self.leftMotor.setSpeed(speed)
		else:
			self.leftMotor.setMode(CLOCKWISE)
			self.leftMotor.setSpeed(-1*speed)
		
	def stop(self):
		self.leftMotor.setMode(1)
		self.rightMotor.setMode(1)
		self.leftMotor.setSpeed(0)
		self.rightMotor.setSpeed(0)
		
	def forward(self):
		# right CW left CCW
		self.leftMotor.setMode(3)
		self.rightMotor.setMode(2)
		self.leftMotor.setSpeed(25)
		self.rightMotor.setSpeed(25)
		
	def backward(self):
		# right CCW left CW
		self.leftMotor.setMode(2)
		self.rightMotor.setMode(3)
		self.leftMotor.setSpeed(25)
		self.rightMotor.setSpeed(25)