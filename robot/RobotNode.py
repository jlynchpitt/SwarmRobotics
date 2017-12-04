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

robotInfo = Robot_Info()
ROBOT_ID = robotInfo.getRobotID()

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
    global wheelSpeed, robot, ROBOT_ID
    
    ########################################################
    #Initialize the node, any subscribers and any publishers
    ########################################################
    rospy.init_node('robot_node', anonymous=True)
    if ROBOT_ID == 1: 
        rospy.Subscriber("/robot_commands_1", WheelSpeeds, updateRobotCommands, queue_size=10)
    elif ROBOT_ID == 2: 
        rospy.Subscriber("/robot_commands_2", WheelSpeeds, updateRobotCommands, queue_size=10)
    elif ROBOT_ID == 3: 
        rospy.Subscriber("/robot_commands_3", WheelSpeeds, updateRobotCommands, queue_size=10)
    elif ROBOT_ID == 4: 
        rospy.Subscriber("/robot_commands_4", WheelSpeeds, updateRobotCommands, queue_size=10)
    
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
        
if __name__ == '__main__':
	robot
	try:
		main()
	except :
		print("exception occurred" + str(sys.exc_info()[0]))
	finally:
		print("cleaning up gpio")
		robot.cleanup()
