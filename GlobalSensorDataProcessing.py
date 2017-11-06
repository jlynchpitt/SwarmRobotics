#!/usr/bin/env python
import roslib
#roslib.load_manifest('rosopencv')
import sys
import rospy
import math
from std_msgs.msg import String
from sensor_msgs.msg import Image
from collections import deque
from swarm.msg import SensorData

global queue, max
queue = deque() ##deque uses FIFO, queue.pop(0) will pop the first element, queue.append() will append to the end
max = 0
currentRed, currentGreen, currentBlue, currentData

########################################################
#Callback functions for each subscriber
#	+ any other custom functions needed
#Do as little processing in the callback as possible
#Try to just store the incoming data + do all
#	processing in the main loop
########################################################
def updateSensorData(data):
	#May be best to store data in a buffer to deal with lots of data
	#	coming in at once from multiple robots
	#Or possibly have each robot publish to a different topic
	#	and have multiple subscibers/callback functions
    queue.append(data)

def main():
    
	########################################################
	#Initialize the node, any subscribers and any publishers
	########################################################
    rospy.init_node('global_sensor_data_processing_node', anonymous=True)
    rospy.Subscriber("/local_sensor_data", SensorData, updateSensorData, queue_size=10)
	globalPub = rospy.Publisher('global_sensor_data', SensorData, queue_size=10)
	
	########################################################
	#Wait here for any data that needs to be ready
	#For data that would crash the program if it was not
	#	ready yet
	########################################################

    while not rospy.is_shutdown():
       
		
		########################################################
		#All code for processing data/algorithm goes here
		########################################################
        currentData = queue.pop(0)
        currentRed = currentData.red
        currentGreen = currentData.green
        currentBlue = currentData.blue
        if(max < currentRed):
            max = currentRed
		
		########################################################
		#Publish data here
		########################################################
        globalPub.publish(max) ##only publishing the global max value currently, will include other data later
        



 
if __name__ == '__main__':
        main()
