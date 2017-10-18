#!/usr/bin/python
import RPi.GPIO as GPIO

class Motor:
	def __init__(self, in1PinNum, in2PinNum, pwmPinNum):
		GPIO.setmode(GPIO.BOARD)
		
		self.in1PinNum = in1PinNum
		GPIO.setup(self.in1PinNum, GPIO.OUT)
		GPIO.output(self.in1PinNum, GPIO.HIGH)
		
		self.in2PinNum = in2PinNum
		GPIO.setup(in2PinNum, GPIO.OUT)
		GPIO.output(self.in2PinNum, GPIO.HIGH)
		
		GPIO.setup(pwmPinNum, GPIO.OUT)
		self.pwmPin = GPIO.PWM(pwmPinNum, 1000) # TODO: Confirm frequency
		self.pwmPin.start(0)
		
	def setMode(self, mode):
		if mode == 0: # Stop motor
			GPIO.output(self.in1PinNum, GPIO.LOW)
			GPIO.output(self.in2PinNum, GPIO.LOW)
		elif mode == 1: # Brake motor
			GPIO.output(self.in1PinNum, GPIO.HIGH)
			GPIO.output(self.in2PinNum, GPIO.HIGH)
		elif mode == 2: # Clockwise
			GPIO.output(self.in1PinNum, GPIO.HIGH)
			GPIO.output(self.in2PinNum, GPIO.LOW)
		else #counter clockwise
			GPIO.output(self.in1PinNum, GPIO.LOW)
			GPIO.output(self.in2PinNum, GPIO.HIGH)
			
	def setSpeed(self, speed): #speed is a percentage from 0-100
		if speed > 100:
			speed = 100
		elif speed < 0:
			speed = 0
		
		self.pwmPin.ChangeDutyCycle(speed)