#!/usr/bin/env python
import roslib
#roslib.load_manifest('rosopencv')
import sys
import rospy
import math
from std_msgs.msg import String
from sensor_msgs.msg import Image
from collections import deque
from swarm.msg import SensorData, RobotVelocity


queue = deque() ##deque uses FIFO, queue.pop(0) will pop the first element, queue.append() will append to the end
currentRed = 0
currentGreen = 0
currentBlue = 0
currentMax = 0
theData = SensorData()
theData.robotID = 1
theData.red = 0
theData.green = 0
theData.blue = 0

cmdVel = RobotVelocity()
cmdVel.x = 25
cmdVel.y = 25
currentMax = 0

########################################################
#Callback functions for each subscriber
#   + any other custom functions needed
#Do as little processing in the callback as possible
#Try to just store the incoming data + do all
#   processing in the main loop
########################################################
def updateSensorData(data):
	global queue, currentRed, currentGreen, currentBlue, currentMax, cmdVel, theData
	#May be best to store data in a buffer to deal with lots of data
    	#   coming in at once from multiple robots
    	#Or possibly have each robot publish to a different topic
    	#   and have multiple subscibers/callback functions
    	#queue.append(data)
	theData = data
def main():
    global queue, currentRed, currentGreen, currentBlue, currentMax, cmdVel, theData 
    ########################################################
    #Initialize the node, any subscribers and any publishers
    ########################################################
    rospy.init_node('global_sensor_data_processing_node', anonymous=True)
    rospy.Subscriber("/local_sensor_data", SensorData, updateSensorData, queue_size=10)
    globalPub = rospy.Publisher('global_sensor_data', SensorData, queue_size=10)
    globalPub2 = rospy.Publisher('suggested_movement', RobotVelocity, queue_size=10)
    
    ########################################################
    #Wait here for any data that needs to be ready
    #For data that would crash the program if it was not
    #   ready yet
    ########################################################

    while not rospy.is_shutdown():
       
        
        ########################################################
        #All code for processing data/algorithm goes here
        ########################################################
        #theData = queue.pop()
        currentRed = theData.red
        currentGreen = theData.green
        currentBlue = theData.blue
        if(currentMax < currentRed):
            currentMax = currentRed

            
        
        ########################################################
        #Publish data here
        ########################################################
        globalPub.publish(theData) ##only publishing the global max value currently, will include other data later
        if(currentRed > 1500):
		cmdVel.x = 0
        	cmdVel.y = 0
	else:
		cmdVel.x = 25
		cmdVel.y = 25
    	globalPub2.publish(cmdVel)
        



 
if __name__ == '__main__':
        main()
