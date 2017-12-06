#!/usr/bin/env python
import roslib
#roslib.load_manifest('rosopencv')
import sys
import rospy
import math
import time
from std_msgs.msg import String
from sensor_msgs.msg import Image
from smbus2 import SMBus
import time
from swarm.msg import SensorData, RobotLocation, RobotLocationList
from Robot_Info import Robot_Info
from copy import deepcopy

data = SensorData()
data.robotID = 0
data.red = 0
data.green = 0
data.blue = 0

locationList = RobotLocationList()
location = RobotLocation()
location.x = 0
location.y = 0
isLocationReady = False

def updateLocation(data):
    global locationList, isLocationReady
    locationList = data
    isLocationReady = True

def main():
    global data, locationList, location, isLocationReady
    robotInfo = Robot_Info()
    robID = robotInfo.getRobotID()
    data.robotID = robID
    sensorAddr =  0x44 ##address of the light sensor
    i2c = SMBus(1) ##initialize SMBus object using the bus number
    i2c.write_byte_data(0x44, 1, 0xD) ##configures the light sensor
    
	########################################################
	#Initialize the node, any subscribers and any publishers
	#Note no subscribers needed for this node
	########################################################
    rospy.init_node('local_sensor_data_node', anonymous=True)
    if(robID == 1):
        pub = rospy.Publisher('local_sensor_data_1', SensorData, queue_size=10)
    elif(robID == 2):
        pub = rospy.Publisher('local_sensor_data_2', SensorData, queue_size=10)
    elif(robID == 3):
        pub = rospy.Publisher('local_sensor_data_3', SensorData, queue_size=10)
    else:
        pub = rospy.Publisher('local_sensor_data_4', SensorData, queue_size=10)
        
    rospy.Subscriber("/robot_location", RobotLocationList, updateLocation, queue_size=10)

    time.sleep(1)
    
    while not isLocationReady:
        pass
	
	########################################################
	#Wait here for any data that needs to be ready
	#For data that would crash the program if it was not
	#	ready yet
	########################################################
    while not rospy.is_shutdown():

	########################################################
	#All code for processing data/algorithm goes here
	########################################################
        #   1. Read in current robot actual angle from location data
        newLocationList = deepcopy(locationList)

        for i in range (0,newLocationList.numRobots):
            if(newLocationList.robotList[i].robotID == ROBOT_ID):
                location = newLocationList.robotList[i]
                foundLocation = True
                #print("Found location for green robot")
                break

        ##reads and compiles the green data
        #greenLow = i2c.read_byte_data(0x44, 9)
        #greenHigh = i2c.read_byte_data(0x44, 10)
        #data.green = greenHigh<<8 | greenLow
        data.green = 0

        ##reads and compiles the red data
        redLow = i2c.read_byte_data(0x44, 11)
        redHigh = i2c.read_byte_data(0x44, 12)
        data.red = redHigh<<8 | redLow

        ##reads and compiles the blue data
        #blueLow = i2c.read_byte_data(0x44, 13)
        #blueHigh = i2c.read_byte_data(0x44, 14)
        #data.blue = blueHigh<<8 | blueLow
        data.blue = 0
        
        ################################################
        #Convert distance location coordinates to coordinate system used by speed
        #   location coord system: 0,0 @ top left of image
        #   speed coord system: 0,0 at center of image
        ################################################
        data.x = location.x - newLocationList.width/2
        data.y = newLocationList.height/2 - location.y
        
	    ########################################################
	    #Publish data here
	    #########################################################
        pub.publish(data)

        time.sleep(1)


if __name__ == '__main__':
        main()
