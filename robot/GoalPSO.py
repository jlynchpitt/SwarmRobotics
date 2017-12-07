#!/usr/bin/env python
import roslib
#roslib.load_manifest('rosopencv')
import sys
import rospy
import math
import random
import time
from copy import deepcopy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from swarm.msg import SensorData, RobotVelocity, RobotLocation, RobotLocationList
from Robot_Info import Robot_Info

cmdVel = RobotVelocity()
cmdVel.x = 25
cmdVel.y = 25

theList = RobotLocationList()

#targetLocation = RobotLocation() ##targetLocation: RobotLocation() of the target robot
#targetLocation.robotID = 0
#targetLocation.x = 0
#targetLocation.y = 0
#targetLocation.angle = 0

currentLocation = RobotLocation() ##currentLocation: RobotLocation() of the current robot
currentLocation.robotID = 0
currentLocation.x = 0
currentLocation.y = 0
currentLocation.angle = 0

theData = SensorData() ##theData: SensorData() of the global data
theData.robotID = 1
theData.red = 0
theData.green = 0
theData.blue = 0

currentData = SensorData() ##theData: SensorData() of the local data
currentData.robotID = 1
currentData.red = 0
currentData.green = 0
currentData.blue = 0

localMaxData = SensorData()
localMaxData.robotID = 1
localMaxData.red = 0
localMaxData.green = 0
localMaxData.blue = 0
localMaxData.x = 0
localMaxData.y = 0

#localMaxPos = RobotLocation()
#localMaxPos.robotID = 1
#localMaxPos.robotColor = ""
#localMaxPos.x = 0
#localMaxPos.y = 0
#localMaxPos.angle = 0

vectorX = 0
vectorY = 0

########################################################
#Callback functions for each subscriber
#	+ any other custom functions needed
#Do as little processing in the callback as possible
#Try to just store the incoming data + do all
#	processing in the main loop
########################################################
def updateLocalData(data):
    global currentData
    currentData = data
	
def updateGlobalData(data):
    global theData
    theData = data
	
def updateLocationList(data):
    global theList
    theList = data

def main():
    global vectorX, vectorY, currentData, theData, theList, localMaxData, currentLocation, cmdVal, foundIt
    robotInfo = Robot_Info()
    robID = robotInfo.getRobotID()
    foundIt = False
	########################################################
	#Initialize the node, any subscribers and any publishers
	########################################################
    rospy.init_node('goal_pso_node', anonymous=True)
    if(robID == 1):
        rospy.Subscriber("/local_sensor_data_1", SensorData, updateLocalData, queue_size=10)
    elif(robID == 2):
        rospy.Subscriber("/local_sensor_data_2", SensorData, updateLocalData, queue_size=10)
    elif(robID == 3):
        rospy.Subscriber("/local_sensor_data_3", SensorData, updateLocalData, queue_size=10)
    elif(robID == 4):
        rospy.Subscriber("/local_sensor_data_4", SensorData, updateLocalData, queue_size=10)
    rospy.Subscriber("/global_sensor_data", SensorData, updateGlobalData, queue_size=10)
    rospy.Subscriber("/robot_location", RobotLocationList, updateLocationList, queue_size=10)
    if(robID == 1):
        pub = rospy.Publisher('suggested_movement_1', RobotVelocity, queue_size=10)
        vectorX = 20
        vectorY = 20
    elif(robID == 2):
        pub = rospy.Publisher('suggested_movement_2', RobotVelocity, queue_size=10)
        vectorX = 20
        vectorY = 0
    elif(robID == 3):
        pub = rospy.Publisher('suggested_movement_3', RobotVelocity, queue_size=10)
        vectorX = -20
        vectorY = 20
    else:
        pub = rospy.Publisher('suggested_movement_4', RobotVelocity, queue_size=10)
        vectorX = 20
        vectorY = -20
    time.sleep(1)
	
	########################################################
	#Wait here for any data that needs to be ready
	#For data that would crash the program if it was not
	#	ready yet
	########################################################
    
    
    while not rospy.is_shutdown():

		########################################################
		#All code for processing data/algorithm goes here
		########################################################
        tempList = deepcopy(theList)
        globalData = deepcopy(theData)
        localData = deepcopy(currentData)

        for i in range (0,tempList.numRobots):
            #if(tempList.robotList[i].robotID == theData.robotID):
            #    targetLocation = deepcopy(tempList.robotList[i])
            if(tempList.robotList[i].robotID == robID):
                currentLocation = deepcopy(tempList.robotList[i])

        if(localData.red > localMaxData.red): ##update the local max and position if the currentData is greater than the local max
            localMaxData = deepcopy(localData)
            #localMaxPos = deepcopy(currentLocation)
            
        ################################################
        #Convert distance location coordinates to coordinate system used by speed
        #   location coord system: 0,0 @ top left of image
        #   speed coord system: 0,0 at center of image
        ################################################
        currentLocation.x = currentLocation.x - tempList.width/2
        currentLocation.y = tempList.height/2 - currentLocation.y
        
        print("glob x: " + str(globalData.x) + " cur x: " + str(currentLocation.x) + " local max x: " + str(localMaxData.x))
        print("vector x: " + str(vectorX))
            
        if(theData.red < 1000 and (not foundIt)): ##case where the global max threshold has not been met, keep searching
            vectorX = vectorX + (2 * random.random() * (globalData.x - currentLocation.x)) + (2 * random.random() * (localMaxData.x - currentLocation.x))
            vectorY = vectorY + (2 * random.random() * (globalData.y - currentLocation.y)) + (2 * random.random() * (localMaxData.y - currentLocation.y))
        elif(theData.red > 1000 and localMaxData.red < 1000 and (not foundIt)): ##case where global max threshold has been found, but this robot isn't there yet 
            vectorX = vectorX + (10 * random.random() * (globalData.x - currentLocation.x)) + 2 * random.random() * (localMaxData.x - currentLocation.x)
            vectorY = vectorY + (10 * random.random() * (globalData.y - currentLocation.y)) + 2 * random.random() * (localMaxData.y - currentLocation.y)
        elif(theData.red > 1000 and localMaxData.red > 1000): ##case where the robot is near the global max threshold, stop it
            vectorX = 0
            vectorY = 0
            foundIt = True

        #Shrink the velocity down to the max velocity vector
        ########################################################
        #Check the commanded velocity and slow it down if 
        #   velocity magnitude greater than desired max speed
        ########################################################
        goalMagnitude = 45
        currentMagnitude = math.sqrt((cmdVel.x**2) + (cmdVel.y**2))
        if currentMagnitude > goalMagnitude:
            magMultiplier = float(goalMagnitude)/float(currentMagnitude)
            vectorX = vectorX*magMultiplier
            vectorY = vectorY*magMultiplier
            
        cmdVel.x = int(vectorX)
        cmdVel.y = int(vectorY)
		
		########################################################
		#Publish data here
		########################################################
        pub.publish(cmdVel)
        
        time.sleep(0.5)


if __name__ == '__main__':
        main()
