#!/usr/bin/env python
import roslib
#roslib.load_manifest('rosopencv')
import sys
import rospy
import math
from std_msgs.msg import String
from sensor_msgs.msg import Image
from swarm.msg import WheelSpeeds

wheelSpeed = WheelSpeeds()
wheelSpeed.leftWheel = 0
wheelSpeed.rightWheel = 0

########################################################
#Callback functions for each subscriber
#	+ any other custom functions needed
#Do as little processing in the callback as possible
#Try to just store the incoming data + do all
#	processing in the main loop
########################################################
def updateRobotCommands(data):
    global wheelSpeed
    wheelSpeed = data
	#Positive wheel speed is moving the wheel in the direction that would move the robot forward
	#Negative wheel speed is moving the wheel in the direction that would move the robot backward
	
def main():
    global wheelSpeed
    
	########################################################
	#Initialize the node, any subscribers and any publishers
	########################################################
    rospy.init_node('robot_node', anonymous=True)
    rospy.Subscriber("/robot_commands", WheelSpeeds, updateRobotCommands, queue_size=10)
	
	robot = Robot_Driver()
	robot.rightWheel(0)
	robot.leftWheel(0)

    while not rospy.is_shutdown():
		########################################################
		#All code for processing data/algorithm goes here
		########################################################
		robot.rightWheel(wheelSpeed.rightWheel)
		robot.leftWheel(wheelSpeed.leftWheel)
		
if __name__ == '__main__':
        main()
