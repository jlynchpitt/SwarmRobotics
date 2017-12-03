#!/usr/bin/env python

########################################################
# Node function:
#   Interface with the actual robot driver/robot to move it
#   Convert wheel speeds to correct clockwise/counterclockwise 
#       rotation for each wheel
#
# Data in: 
#   commanded wheel speeds
########################################################

import roslib
import time
#roslib.load_manifest('rosopencv')
import sys
import rospy
import math
from swarm.msg import WheelSpeeds
from Robot_Driver import Robot_Driver
from Robot_Info import Robot_Info

robot = Robot_Driver()

wheelSpeed = WheelSpeeds()
wheelSpeed.leftWheel = 0
wheelSpeed.rightWheel = 0

########################################################
#Callback functions for each subscriber
#   + any other custom functions needed
#Do as little processing in the callback as possible
#Try to just store the incoming data + do all
#   processing in the main loop
########################################################
def updateRobotCommands(data):
    global wheelSpeed
    wheelSpeed = data
    #Positive wheel speed is moving the wheel in the direction that would move the robot forward
    #Negative wheel speed is moving the wheel in the direction that would move the robot backward
    
def main():
    global wheelSpeed, robot
    
    ########################################################
    #Initialize the node, any subscribers and any publishers
    ########################################################
    rospy.init_node('robot_node', anonymous=True)
    rospy.Subscriber("/robot_commands", WheelSpeeds, updateRobotCommands, queue_size=10)
    
    robot.rightWheel(0)
    robot.leftWheel(0)
	
    #Test printing out robot ID
    #robotInfo = Robot_Info()
    #id = robotInfo.getRobotID()
    #print(id)
    while not rospy.is_shutdown():
        ########################################################
        #All code for processing data/algorithm goes here
        ########################################################
        robot.rightWheel(wheelSpeed.rightWheel)
        robot.leftWheel(wheelSpeed.leftWheel)
        time.sleep(.1)
        
        
if __name__ == '__main__':
	robot
	try:
		main()
	except :
		print("exception occurred" + str(sys.exc_info()[0]))
	finally:
		print("cleaning up gpio")
		robot.cleanup()
