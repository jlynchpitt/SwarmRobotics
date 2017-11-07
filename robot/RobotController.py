#!/usr/bin/env python

########################################################
# Node function:
#   Convert X and Y commanded velocities into wheel speeds
#   1. Calculate vector angle + speed from XY velocities
#   2. Calculate desired wheel speed
#       (identical wheel speed for each wheel if robot 
#       was moving forward in a straight line)
#   3. Read in current robot actual angle from location data
#   4. Use PID to calculate difference in wheel speeds to turn 
#       robot to desired angle
#   5. Publish calculated wheel speeds
# Data in: 
#   location data from central hub
#   commanded XY velocities
# Data out:
#   wheel speeds
########################################################


import roslib
#roslib.load_manifest('rosopencv')
import sys
import rospy
import math
from swarm.msg import RobotVelocity, WheelSpeeds

cmdVel = RobotVelocity()
cmdVel.x = 0
cmdVel.y = 0
wheelSpeed = WheelSpeeds()
wheelSpeed.rightWheel = 0
wheelSpeed.leftWheel = 0

########################################################
#Callback functions for each subscriber
#   + any other custom functions needed
#Do as little processing in the callback as possible
#Try to just store the incoming data + do all
#   processing in the main loop
########################################################
def updateCommandedMovement(data):
    global cmdVel
    cmdVel = data
    
def updateLocation(data):
    global colorImage, isColorImageReady, ratio
    colorImage = data
    isColorImageReady = True

def main():
    global cmdVel, wheelSpeed
    
    ########################################################
    #Initialize the node, any subscribers and any publishers
    #TODO: Change data types of subscribers
    ########################################################
    rospy.init_node('robot_controller_node', anonymous=True)
    rospy.Subscriber("/commandedMovement", RobotVelocity, updateCommandedMovement, queue_size=10)
    #rospy.Subscriber("/location_data", Image, updateLocation, queue_size=10)
    pub = rospy.Publisher('robot_commands', WheelSpeeds, queue_size=10)
    
    ########################################################
    #Wait here for any data that needs to be ready
    #For data that would crash the program if it was not
    #   ready yet
    ########################################################
    while not rospy.is_shutdown():
        ########################################################
        #All code for processing data/algorithm goes here
        ########################################################
        
        # 1. Calculate vector magnitude + angle from XY velocities
        # 2. Calculate desired wheel speed (vMagnitude) 
        #       (identical wheel speed for each wheel if robot was moving forward in a straight line)
        vMagnitude = math.sqrt(cmdVel.x^2 + cmdVel.y^2)
        
        #limit vMagnitude to 0-100 since wheel speeds are a percentage of max speed
        if vMagnitude > 100:
            vMagnitude = 100
        
        #TODO: Confirm this calculation is correct
        #If python 2.7 may do integer division for y/x
	tanDivAmount = cmdVel.y
	if cmdVel.x != 0:
		tanDivAmount = cmdVel.y/cmdVel.x
        vAngle = math.degrees(math.atan(tanDivAmount))
        
        # 3. Read in current robot actual angle from location data
        # 4. Use PID to calculate difference in wheel speeds to turn 
        #       robot to desired angle
        
        # 5. Publish calculated wheel speeds
        wheelSpeed.rightWheel = vMagnitude
        wheelSpeed.leftWheel = vMagnitude
        
        pub.publish(wheelSpeed)
 
if __name__ == '__main__':
        main()
