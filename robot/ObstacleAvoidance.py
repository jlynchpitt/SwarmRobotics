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
import time
import math
from copy import deepcopy
from Robot_Info import Robot_Info
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

robotInfo = Robot_Info()
ROBOT_ID = robotInfo.getRobotID()
print(ROBOT_ID)
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
    
    time.sleep(1)

    while not isLocationReady:
        pass
        
    centerX = float(locationList.width)/2
    centerY = float(locationList.height)/2
    
    while not rospy.is_shutdown():
        ########################################################
        #All code for processing data/algorithm goes here
        ########################################################
        #Initialize to suggested velocity in case no corrective action needs to be taken
        newLocationList = deepcopy(locationList)
        newSugVel = deepcopy(sugVel)
	
        cmdVel.x = newSugVel.x
        cmdVel.y = newSugVel.y
        foundLocation = False
        
        #   1. Read in current robot actual angle from location data
        for i in range (0,newLocationList.numRobots):
            if(newLocationList.robotList[i].robotID == ROBOT_ID):
                location = newLocationList.robotList[i]
                foundLocation = True
                #print("Found location for green robot")
                break
        
        #   2. Determine if robot close to edge of frame
        if(foundLocation == True):
            BUFFER = 0.25 # In meters
            robotOnEdge = False
            if location.y < BUFFER or location.x < BUFFER or (newLocationList.width - location.x) < BUFFER or (newLocationList.height - location.y) < BUFFER:
                robotOnEdge = True
                goalVelocityMagnitude = 30
                x = centerX - location.x
                y = location.y - centerY
                
                currentMagnitude = math.sqrt((x*x) + (y*y))
                print("current mag: " + str(currentMagnitude))
                magMultiplier = float(goalVelocityMagnitude)/float(currentMagnitude)
                print("multiplier: " + str(magMultiplier))
                cmdVel.x = x*magMultiplier
                cmdVel.y = y*magMultiplier
                print("vector x: " + str(x) + " vector y: " + str(y))
                print("new x: " + str(cmdVel.x) + " new y: " + str(cmdVel.y))
                
            #   3a. If close to edge direct robot to center of frame
            #   3b. If not close to edge pass along suggested movement
            #   4. Determine field of vision where collision may be imenent
            #Create rectangle
            W = 0.5 #meters
            H = 0.25 #meters
            #x1 = W/2
            #y1 = 0
            #x2 = x1
            #y2 = H
            #x3 = -1*w/2
            #y3 = y2
            #x4 = x3
            #y4 = 0
            #x1 = location.x + W/2
            #y1 = location.y
            #x2 = x1
            #y2 = location.y + H
            #x3 = location.x - w/2
            #y3 = y2
            #x4 = x3
            #y4 = location.y
            
            #Create rectangle - polar coordinates
            r1 = W/2
            r2 = math.sqrt((W**2)/4+H**2)
            r3 = r2
            r4 = -180
            theta1 = 0
            theta2 = math.degrees(math.acos((W/2)/r2))
            theta3 = 180-theta2
            theta4 = -180
            
            #Rotate angle
            theta1 = theta1 + location.angle
            theta2 = theta2 + location.angle
            theta3 = theta3 + location.angle
            theta4 = theta4 + location.angle
            
            #Calculate new x and y
            #rcostheta
            x1 = r1*math.cos(math.radians(theta1))
            y1 = r1*math.sin(math.radians(theta1))
            A = Point(x1, y1)
            x2 = r2*math.cos(math.radians(theta2))
            y2 = r2*math.sin(math.radians(theta2))
            B = Point(x2, y2)
            x3 = r3*math.cos(math.radians(theta3))
            y3 = r3*math.sin(math.radians(theta3))
            C = Point(x3, y3)
            x4 = r4*math.cos(math.radians(theta4))
            y4 = r4*math.sin(math.radians(theta4))
            D = Point(x4, y4)

            #   5a. If any other robots in "field of vision" - stop robot or change desired velocity
            #   5b. If no other robots in "field of "vision - pass along suggested movement command
            for i in range (0,newLocationList.numRobots):
                if(newLocationList.robotList[i].robotID != ROBOT_ID):
                    P = Point(newLocationList.robotList[i].x, newLocationList.robotList[i].y)
                    if(is_inside(P, A, B, C, D)):
                        print("Obstacle in sight")
                    break
        else:
            cmdVel.x = 0
            cmdVel.y = 0
        
        ########################################################
        #Publish data here
        ########################################################
        pub.publish(cmdVel)
        time.sleep(0.05)
        
#class Point has two variables:x and y.
def vec(A,B): #vector of point A,B
    return Point(B.x-A.x,B.y-A.y)

def dot(P,Q): #scalar product of two vectors
    return P.x*Q.x+P.y*Q.y

def is_inside(P,A,B,C,D): #P is the given point,others are 4 vertices 
    return 0<=dot(vec(A,B),vec(A,P))<=dot(vec(A,B),vec(A,B)) and \
       0<=dot(vec(B,C),vec(B,P))<=dot(vec(B,C),vec(B,C))
       
class Point:
    """ Point class represents and manipulates x,y coords. """

    def __init__(self, xx, yy):
        """ Create a new point at the origin """
        self.x = xx
        self.y = yy

 
if __name__ == '__main__':
        main()
