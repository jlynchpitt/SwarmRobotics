#!/usr/bin/python

########################################################
# Class function:
#	Driver for sending commands to the motor driver/motors
########################################################

import os.path
#Green - r1
#Red - r3
#Blue - r4
#Yellow - r2

class Robot_Info:
	def getRobotID(self):
		if os.path.exists('/home/pi/catkin_ws/src/swarm/robot/1.txt') == True:
			return 1
		elif os.path.exists('/home/pi/catkin_ws/src/swarm/robot/2.txt') == True:
			return 2
		elif os.path.exists('/home/pi/catkin_ws/src/swarm/robot/3.txt') == True:
			return 3
		else:
			return 4
