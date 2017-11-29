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
from swarm.msg import RobotVelocity, WheelSpeeds, RobotLocationList, RobotLocation

cmdVel = RobotVelocity()
cmdVel.x = 0
cmdVel.y = 0
wheelSpeed = WheelSpeeds()
wheelSpeed.rightWheel = 0
wheelSpeed.leftWheel = 0
locationList = RobotLocationList()
location = RobotLocation()

ROBOT_ID = 2

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
    global locationList
    locationList = data

def main():
    global cmdVel, wheelSpeed, pid, locationList, location
    
    ########################################################
    #Initialize the node, any subscribers and any publishers
    #TODO: Change data types of subscribers
    ########################################################
    rospy.init_node('robot_controller_node', anonymous=True)
    rospy.Subscriber("/commanded_movement", RobotVelocity, updateCommandedMovement, queue_size=10)
    rospy.Subscriber("/location_data", RobotLocationList, updateLocation, queue_size=10)
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
        vMagnitude = math.sqrt(math.pow(cmdVel.x,2) + math.pow(cmdVel.y,2))
        
        #limit vMagnitude to 0-100 since wheel speeds are a percentage of max speed
        if vMagnitude > 100:
            vMagnitude = 100
        
	    print("cmd x: " + str(cmdVel.x) + " cmd y: " + str(cmdVel.y))
	    print("velocity magnitude: " + str(vMagnitude))

        #TODO: Confirm this calculation is correct
        #If python 2.7 may do integer division for y/x
	    tanDivAmount = cmdVel.y
	    if cmdVel.x != 0:
		    tanDivAmount = cmdVel.y/cmdVel.x
            vAngle = math.degrees(math.atan(tanDivAmount))
        elif cmdVel.y >= 0:
            vAngle = 90
        else:
            vAngle = -90
        
        # 3. Read in current robot actual angle from location data
        for i in range (0,locationList.numRobots):
            if(locationList.robotList[i].robotID == ROBOT_ID):
                location = locationList.robotList[i]
                print("Found location for green robot")
                break
                
        # 4. Calculate difference in wheel speeds to turn robot to desired angle
        #How much to turn and what direction
        angleToTurn = 0
        clockWise = True
        
        currentAngle = location.angle
        angleDiff = vAngle - currentAngle
        angleDiffABS = math.fabs(angleDiff)

        wheelDiff = 0
        DEADZONE = 5 #+- angle to not turn anymore
        if angleDiffABS < DEADZONE or angleDiffABS > (360-DEADZONE):
            if angleDiff >= 0 and angleDiff <= 180:
                angleToTurn = angleDiff
                clockWise = False
            elif angleDiff <= 0 and angleDiff >= -180:
                angleToTurn = angleDiffABS
                clockWise = True
            elif angleDiff > 180 and angleDiff < 360:
                angleToTurn = 360 - angleDiff
                clockWise = True
            else: #-181 ->> -360
                angleToTurn = 360 + angleDiff
                clockWise = False
                
            if angleToTurn < 45:
                wheelDiff = 5
            elif angleToTurn < 90
                wheelDiff = 10
            else:
                wheelDiff = 15
            
        # clockwise increase left wheel speed
        # counterclockwise - increase right wheel speed
        multiplier = -1
        if(clockWise == True):
            multiplier = 1

        vMagnitude = 0 #TODO: This is temporary for testing
        
        # 5. Publish calculated wheel speeds
        wheelSpeed.rightWheel = vMagnitude + wheelDiff * multiplier * -1
        wheelSpeed.leftWheel = vMagnitude + wheelDiff * multiplier

        pub.publish(wheelSpeed)
 
if __name__ == '__main__':
        main()
