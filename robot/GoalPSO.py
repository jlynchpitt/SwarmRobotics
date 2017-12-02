#!/usr/bin/env python
import roslib
#roslib.load_manifest('rosopencv')
import sys
import rospy
import math
import random
from std_msgs.msg import String
from sensor_msgs.msg import Image
from swarm.msg import SensorData, RobotVelocity, RobotLocation, RobotLocationList
from Robot_Info import Robot_Info

cmdVel = RobotVelocity()
cmdVel.x = 25
cmdVel.y = 25

theList = RobotLocationList()

targetLocation = RobotLocation() ##targetLocation: RobotLocation() of the target robot
targetLocation.robotID = 1
targetLocation.robotColor = ""
targetLocation.x = 0
targetLocation.y = 0
targetLocation.angle = 0

currentLocation = RobotLocation() ##currentLocation: RobotLocation() of the current robot
currentLocation.robotID = 1
currentLocation.robotColor = ""
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

localMaxPos = RobotLocation()
localMaxPos.robotID = 1
localMaxPos.robotColor = ""
localMaxPos.x = 0
localMaxPos.y = 0
localMaxPos.angle = 0

########################################################
#Callback functions for each subscriber
#	+ any other custom functions needed
#Do as little processing in the callback as possible
#Try to just store the incoming data + do all
#	processing in the main loop
########################################################
def updateLocalData(data):
    currentData = data
	
def updateGlobalData(data):
    theData = data
	
def updateLocationList(data):
    theList = data

def main():
    global vectorX, vectorY
    global counter = 0
	########################################################
	#Initialize the node, any subscribers and any publishers
	########################################################
    rospy.init_node('goal_pso_node', anonymous=True)
    rospy.Subscriber("/local_sensor_data", SensorData, updateLocalData, queue_size=10)
    rospy.Subscriber("/global_sensor_data", SensorData, updateGlobalData, queue_size=10)
    rospy.Subscriber("/robot_location", RobotLocationList, updateLocationList, queue_size=10)
    pub = rospy.Publisher('suggested_movement', RobotVelocity, queue_size=10)
	
	########################################################
	#Wait here for any data that needs to be ready
	#For data that would crash the program if it was not
	#	ready yet
	########################################################

	robotInfo = Robot_Info()
	robID = robotInfo.getRobotID
	
	
    while not rospy.is_shutdown():

		
		########################################################
		#All code for processing data/algorithm goes here
		########################################################

        if(len(theList) > 1):
            for ele in theList: ##searches the list for the robotLccation with the ID matching the one sent through globalData
                if ele.robotID == theData.robotID:
                    targetLocation = ele
        else:
            targetLocation = theList[0]

        if(len(theList) > 1):
            for ele in theList: ##searches the list for the robotLocatoin with the ID matching the currentData
                if ele.robotID == robID:
                    currentLocation = ele
        else:
            currentLocation = theList[0]

        if(currentData.red > localMaxData.red): ##update the local max and position if the currentData is greater than the local max
            localMaxData = currentData
            localMaxPos = currentLocation

        if(theData.red < 1500): ##case where the global max threshold has not been met, keep searching
            vectorX = vectorX + 2 * random.random() * (targetLocation.x - currentLocation.x) + 2 * random.random() * (targetLocation.x - localMaxPos.x)
            vectorY = vectorY + 2 * random.random() * (targetLocation.y - currentLocation.y) + 2 * random.random() * (targetLocation.y - localMaxPos.y)
        elif(theData.red > 1500 && currentData.red < 1500): ##case where global max threshold has been found, but this robot isn't there yet 
            vectorX = vectorX + 2 * random.random() * (targetLocation.x - currentLocation.x) + 2 * random.random() * (targetLocation.x - localMaxPos.x)
            vectorY = vectorY + 2 * random.random() * (targetLocation.y - currentLocation.y) + 2 * random.random() * (targetLocation.y - localMaxPos.y)
        elif(theData.red > 1500 && currentData.red > 1500) ##case where the robot is near the global max threshold, stop it
            vectorX = 0
            vectorY = 0

        cmdVel.x = vectorX
        cmdVel.y = vectorY
		
		########################################################
		#Publish data here
		########################################################
        pub.publish(cmdVel)
        


##read in globalData, has robotID. Use ID to search the list for the robot location data. Use data to determine an x and y vector.
#TODO I get the RobotID from the lcoal data, is this accurate? or is there something else I should subscribe to?
 
if __name__ == '__main__':
        main()
