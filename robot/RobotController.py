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
import time
from copy import deepcopy
from Robot_Info import Robot_Info
from swarm.msg import RobotVelocity, WheelSpeeds, RobotLocationList, RobotLocation

cmdVel = RobotVelocity()
cmdVel.x = 0
cmdVel.y = 0
wheelSpeed = WheelSpeeds()
wheelSpeed.rightWheel = 0
wheelSpeed.leftWheel = 0
locationList = RobotLocationList()
location = RobotLocation()

robotInfo = Robot_Info()
ROBOT_ID = robotInfo.getRobotID()

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
    global cmdVel, wheelSpeed, pid, locationList, location, ROBOT_ID
    
    ########################################################
    #Initialize the node, any subscribers and any publishers
    #TODO: Change data types of subscribers
    ########################################################
    rospy.init_node('robot_controller_node', anonymous=True)
    if ROBOT_ID == 1:
        rospy.Subscriber("/commanded_movement_1", RobotVelocity, updateCommandedMovement, queue_size=10)
    elif ROBOT_ID == 2:
        rospy.Subscriber("/commanded_movement_2", RobotVelocity, updateCommandedMovement, queue_size=10)
    elif ROBOT_ID == 3:
        rospy.Subscriber("/commanded_movement_3", RobotVelocity, updateCommandedMovement, queue_size=10)
    else:
        rospy.Subscriber("/commanded_movement_4", RobotVelocity, updateCommandedMovement, queue_size=10)
    rospy.Subscriber("/robot_location", RobotLocationList, updateLocation, queue_size=10)
    if ROBOT_ID == 1:
        pub = rospy.Publisher('robot_commands_1', WheelSpeeds, queue_size=10)
    elif ROBOT_ID == 2:
        pub = rospy.Publisher('robot_commands_2', WheelSpeeds, queue_size=10)
    elif ROBOT_ID == 3:
        pub = rospy.Publisher('robot_commands_3', WheelSpeeds, queue_size=10)
    else:
        pub = rospy.Publisher('robot_commands_4', WheelSpeeds, queue_size=10)
    
    time.sleep(1)
    
    ########################################################
    #Wait here for any data that needs to be ready
    #For data that would crash the program if it was not
    #   ready yet
    ########################################################
    while not rospy.is_shutdown():
        ########################################################
        #All code for processing data/algorithm goes here
        ########################################################
        #Define/initialize variables
        vMagnitude = 0
        wheelDiff = 0
        clockWise = True
        newCmdVel = deepcopy(cmdVel)
        newLocationList = deepcopy(locationList)
	
        if(newCmdVel.rightObstacle == False and newCmdVel.leftObstacle == False):
            # 1. Calculate vector magnitude + angle from XY velocities
            # 2. Calculate desired wheel speed (vMagnitude) 
            #       (identical wheel speed for each wheel if robot was moving forward in a straight line)
            vMagnitude = math.sqrt(math.pow(newCmdVel.x,2) + math.pow(newCmdVel.y,2))

            #limit vMagnitude to 0-100 since wheel speeds are a percentage of max speed
            if vMagnitude > 100:
                vMagnitude = 100

            #print("cmd x: " + str(cmdVel.x) + " cmd y: " + str(cmdVel.y))
            #print("velocity magnitude: " + str(vMagnitude))

            #TODO: Confirm this calculation is correct
            #If python 2.7 may do integer division for y/x
            x = newCmdVel.x
            y = newCmdVel.y
            vAngle = 0
            if x >= 0 and y == 0:
                vAngle = 0
            elif x == 0 and y > 0:
                vAngle = 90
            elif x < 0 and y == 0:
                vAngle = 180
            elif x == 0 and y < 0:
                vAngle = 270
            elif x > 0 and y > 0: #quadrant I
                vAngle = math.degrees(math.atan(float(y)/float(x)))
            elif x < 0: #quadrant II + 3
                vAngle = 180 + math.degrees(math.atan(float(y)/float(x)))
            else: #quadrant IV
                vAngle = 360 + math.degrees(math.atan(float(y)/float(x)))

            # 3. Read in current robot actual angle from location data
            for i in range (0,newLocationList.numRobots):
                if(newLocationList.robotList[i].robotID == ROBOT_ID):
                    location = newLocationList.robotList[i]
                    #print("Found location for green robot")
                    break

            # 4. Calculate difference in wheel speeds to turn robot to desired angle
            #How much to turn and what direction
            angleToTurn = 0

            currentAngle = location.angle
            angleDiff = vAngle - currentAngle
            angleDiffABS = math.fabs(angleDiff)
            print("current angle: " +str(currentAngle) + " target: " + str(vAngle) + " diff: " + str(angleDiff))
            wheelDiff = 0
            DEADZONE = 5 #+- angle to not turn anymore
            if angleDiffABS > DEADZONE and angleDiffABS < (360-DEADZONE):
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
                    wheelDiff = 9
                elif angleToTurn < 90:
                    wheelDiff = 20
                else:
                    wheelDiff = 30

            # Set wheelDiff + velocity if current goal velocities both 0
            if vMagnitude == 0:
                wheelDiff = 0
        elif newCmdVel.rightObstacle == True: #Obstacle - no forward motion just turn
            clockWise = True
            vMagnitude = 0
            wheelDiff = 20
        else:
            #Left Obstacle - turn right - clockwise
            clockWise = True
            vMagnitude = 0
            wheelDiff = 20

        # clockwise increase left wheel speed
        # counterclockwise - increase right wheel speed
        multiplier = -1
        if(clockWise == True):
            multiplier = 1

        #vMagnitude = 0 #TODO: This is temporary for testing
        
        # 5. Publish calculated wheel speeds
        newRightWheel = vMagnitude + wheelDiff * multiplier * -1
        newLeftWheel = vMagnitude + wheelDiff * multiplier
        
        if newRightWheel > 100:
            newRightWheel = 100
        elif newRightWheel < -100:
            newRightWheel = -100
        
        if newLeftWheel > 100:
            newLeftWheel = 100
        elif newLeftWheel < -100:
            newLeftWheel = -100
        
        wheelSpeed.rightWheel = newRightWheel
        wheelSpeed.leftWheel = newLeftWheel
        #print("wheel diff: " + str(wheelDiff))
        #print("right: " + str(wheelSpeed.rightWheel) + " left: " + str(wheelSpeed.leftWheel))
        pub.publish(wheelSpeed)
        time.sleep(.075)
 
if __name__ == '__main__':
        main()
