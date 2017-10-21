#!/usr/bin/python
import RPi.GPIO as GPIO
from Motor import Motor

class Robot:
	def __init__(self):
		#Init standby pin to high
		GPIO.setmode(GPIO.BOARD)
		GPIO.setup(15, GPIO.OUT)
		GPIO.output(15, GPIO.HIGH)

		self.leftMotor = Motor(29, 31, 33)
		self.rightMotor = Motor(13, 11, 12)
		
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