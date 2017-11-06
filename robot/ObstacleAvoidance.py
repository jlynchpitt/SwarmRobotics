#!/usr/bin/env python

########################################################
# Node function:
#	Ensure robots will not crash into each other
#	1. Read in current robot actual angle from location data
#	2. Determine field of vision where collision may be imenent
#	3a. If any other robots in "field of vision" - stop robot or change desired velocity
#	3b. If no other robots in "field of "vision - pass along suggested movement command 
# Data in: 
#	location data from central hub
#	suggested XY velocities from algorithm hub
# Data out:
#	commanded XY velocities
########################################################

import roslib
#roslib.load_manifest('rosopencv')
import sys
import rospy
import math
from swarm.msg import RobotVelocity

sugVel = RobotVelocity()
sugVel.x = 0
sugVel.y = 0
cmdVel = RobotVelocity()
cmdVel.x = 0
cmdVel.y = 0

########################################################
#Callback functions for each subscriber
#	+ any other custom functions needed
#Do as little processing in the callback as possible
#Try to just store the incoming data + do all
#	processing in the main loop
########################################################
def updateSuggestedMovemement(data):
    global sugVel
    sugVel = data
	
def updateLocation(data):
    global colorImage, isColorImageReady, ratio
    colorImage = data
    isColorImageReady = True

def main():
    global sugVel, cmdVel
    
	########################################################
	#Initialize the node, any subscribers and any publishers
	########################################################
    rospy.init_node('obstacle_avoidance_node', anonymous=True)
    rospy.Subscriber("/suggested_movement", RobotVelocity, updateSuggestedMovement, queue_size=10)
    rospy.Subscriber("/location_data", Image, updateLocation, queue_size=10)
	pub = rospy.Publisher('commanded_movement', RobotVelocity, queue_size=10)
	
    while not rospy.is_shutdown():
		########################################################
		#All code for processing data/algorithm goes here
		########################################################
        #	1. Read in current robot actual angle from location data
		#	2. Determine field of vision where collision may be imenent
		#	3a. If any other robots in "field of vision" - stop robot or change desired velocity
		#	3b. If no other robots in "field of "vision - pass along suggested movement command 
		cmdVel.x = sugVel.x
		cmdVel.y = sugVel.y
		
		########################################################
		#Publish data here
		########################################################
        pub.publish(cmdVel)
        



 
if __name__ == '__main__':
        main()
