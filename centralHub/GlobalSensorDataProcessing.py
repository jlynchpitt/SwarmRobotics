#!/usr/bin/env python
import roslib
#roslib.load_manifest('rosopencv')
import sys
import rospy
import math
import time
from std_msgs.msg import String
from sensor_msgs.msg import Image
from collections import deque
from swarm.msg import SensorData, RobotLocation, RobotLocationList
from copy import deepcopy


currentRed = 0
currentGreen = 0
currentBlue = 0
currentMax = 0

theData1 = SensorData()
theData1.robotID = 1
theData1.red = 0
theData1.green = 0
theData1.blue = 0
theData1.x = 0
theData1.y = 0

theData2 = SensorData()
theData2.robotID = 2
theData2.red = 0
theData2.green = 0
theData2.blue = 0
theData2.x = 0
theData2.y = 0

theData3 = SensorData()
theData3.robotID = 3
theData3.red = 0
theData3.green = 0
theData3.blue = 0
theData3.x = 0
theData3.y = 0

theData4 = SensorData()
theData4.robotID = 4
theData4.red = 0
theData4.green = 0
theData4.blue = 0
theData4.x = 0
theData4.y = 0

maxData = SensorData()
maxData.robotID = 1
maxData.red = 0
maxData.green = 0
maxData.blue = 0
maxData.x = 0
maxData.y = 0

theList = RobotLocationList()




########################################################
#Callback functions for each subscriber
#   + any other custom functions needed
#Do as little processing in the callback as possible
#Try to just store the incoming data + do all
#   processing in the main loop
########################################################
def updateSensorData1(data):
	global theData1
	#May be best to store data in a buffer to deal with lots of data
    	#   coming in at once from multiple robots
    	#Or possibly have each robot publish to a different topic
    	#   and have multiple subscibers/callback functions
    	#queue.append(data)
	theData1 = data
def updateSensorData2(data):
        global theData2
        theData2 = data
def updateSensorData3(data):
        global theData3
        theData3 = data
def updateSensorData4(data):
        global theData4
        theData4 = data

def main():
    global  currentRed, currentGreen, currentBlue, currentMax, theData1, theData2, theData3, theData4, theList, maxData
    ########################################################
    #Initialize the node, any subscribers and any publishers
    ########################################################
    rospy.init_node('global_sensor_data_processing_node', anonymous=True)
    rospy.Subscriber("/local_sensor_data_1", SensorData, updateSensorData1, queue_size=10)
    rospy.Subscriber("/local_sensor_data_2", SensorData, updateSensorData2, queue_size=10)
    rospy.Subscriber("/local_sensor_data_3", SensorData, updateSensorData3, queue_size=10)
    rospy.Subscriber("/local_sensor_data_4", SensorData, updateSensorData4, queue_size=10)
    sensorPub = rospy.Publisher('global_sensor_data', SensorData, queue_size=10)
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
        if(theData1.red > maxData.red):
            maxData = deepcopy(theData1)
        if(theData2.red > maxData.red):
            maxData = deepcopy(theData2)
        if(theData3.red > maxData.red):
            maxData = deepcopy(theData3)
        if(theData4.red > maxData.red):
            maxData = deepcopy(theData4)

        
        ########################################################
        #Publish data here
        ########################################################
        sensorPub.publish(maxData)
        time.sleep(1)
        


if __name__ == '__main__':
        main()
