#!/usr/bin/env python

########################################################
# Node function:
#   Ensure robots will not crash into each other
#   1. Read in current robot actual angle from location data
#   2. Determine field of vision where collision may be imenent
#   3a. If any other robots in "field of vision" - stop robot or change desired velocity
#   3b. If no other robots in "field of "vision - pass along suggested movement command 
# Data in: 
#   location data from central hub
#   suggested XY velocities from algorithm hub
# Data out:
#   commanded XY velocities
########################################################

import roslib
#roslib.load_manifest('rosopencv')
import sys
import rospy
import math
from swarm.msg import RobotVelocity, RobotLocationList, RobotLocation

sugVel = RobotVelocity()
sugVel.x = 0
sugVel.y = 0
cmdVel = RobotVelocity()
cmdVel.x = 0
cmdVel.y = 0
locationList = RobotLocationList()
location = RobotLocation()
isLocationReady = False

ROBOT_ID = 2

########################################################
#Callback functions for each subscriber
#   + any other custom functions needed
#Do as little processing in the callback as possible
#Try to just store the incoming data + do all
#   processing in the main loop
########################################################
def updateSuggestedMovement(data):
    global sugVel
    sugVel = data
    
def updateLocation(data):
    global locationList, isLocationReady
    locationList = data
    isLocationReady = True

def main():
    global sugVel, cmdVel, locationList, location, isLocationReady
    
    ########################################################
    #Initialize the node, any subscribers and any publishers
    ########################################################
    rospy.init_node('obstacle_avoidance_node', anonymous=True)
    rospy.Subscriber("/suggested_movement", RobotVelocity, updateSuggestedMovement, queue_size=10)
    rospy.Subscriber("/robot_location", RobotLocationList, updateLocation, queue_size=10)
    pub = rospy.Publisher('commanded_movement', RobotVelocity, queue_size=10)
    
    while not isLocationReady:
        pass
        
    centerX = float(locationList.width)/2
    centerY = float(locationList.height)/2
    
    while not rospy.is_shutdown():
        ########################################################
        #All code for processing data/algorithm goes here
        ########################################################
        #Initialize to suggested velocity in case no corrective action needs to be taken
        cmdVel.x = sugVel.x
        cmdVel.y = sugVel.y
        foundLocation = False
        
        #   1. Read in current robot actual angle from location data
        for i in range (0,locationList.numRobots):
            if(locationList.robotList[i].robotID == ROBOT_ID):
                location = locationList.robotList[i]
                foundLocation = True
                #print("Found location for green robot")
                break
        
        #   2. Determine if robot close to edge of frame
        if(foundLocation == True):
            BUFFER = 0.1 # In meters
            robotOnEdge = False
            if location.y < BUFFER or location.x < BUFFER or (locationList.width - location.x) < BUFFER or (locationList.height - location.y) < BUFFER:
		robotOnEdge = True
                goalVelocityMagnitude = 50
                x = centerX - location.x
                y = location.y - centerY
                
                currentMagnitude = math.sqrt(x**2 + y**2)
                magMultiplier = math.sqrt(goalVelocityMagnitude/currentMagnitude)
                cmdVel.x = x*magMultiplier
                cmdVel.y = y*magMultiplier
		print("new x: " + str(cmdVel.x) + " new y: " + str(cmdVel.y))
                
            #   3a. If close to edge direct robot to center of frame
            #   3b. If not close to edge pass along suggested movement
            #   4. Determine field of vision where collision may be imenent
            #   5a. If any other robots in "field of vision" - stop robot or change desired velocity
            #   5b. If no other robots in "field of "vision - pass along suggested movement command 
        else:
            cmdVel.x = 0
            cmdVel.y = 0
        
        ########################################################
        #Publish data here
        ########################################################
        pub.publish(cmdVel)
        



 
if __name__ == '__main__':
        main()
