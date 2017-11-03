#!/usr/bin/env python
import roslib
#roslib.load_manifest('rosopencv')
import sys
import rospy
import math
from std_msgs.msg import String
from sensor_msgs.msg import Image
from collections import deque

global queue, max
queue = deque() ##deque uses FIFO, queue.pop(0) will pop the first element, queue.append() will append to the end
max = 0
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
    rospy.Subscriber("/local_sensor_data", Image, updateSensorData, queue_size=10)
	globalPub = rospy.Publisher('global_sensor_data', Image, queue_size=10)
	
	########################################################
	#Wait here for any data that needs to be ready
	#For data that would crash the program if it was not
	#	ready yet
	########################################################
    while not isColorImageReady:
        pass

    while not rospy.is_shutdown():
        try:
            color_image = bridge.imgmsg_to_cv2(colorImage, "bgr8")
        except CvBridgeError, e:
            print e
            print "colorImage"
		
		########################################################
		#All code for processing data/algorithm goes here
		########################################################
        current = queue.pop(0)
        if(current > max):
            max = current
            print("Current max is " + max)
		
		########################################################
		#Publish data here
		########################################################
        globalPub.publish(max)
        



 
if __name__ == '__main__':
        main()
