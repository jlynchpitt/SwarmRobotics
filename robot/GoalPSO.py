#!/usr/bin/env python
import roslib
#roslib.load_manifest('rosopencv')
import sys
import rospy
import math
from std_msgs.msg import String
from sensor_msgs.msg import Image
from swarm.msg import SensorData, RobotVelocity

counter = 0
cmdVel = RobotVelocity()
cmdVel.x = 25
cmdVel.y = 25

theData = SensorData()
theData.robotID = 1
theData.red = 0
theData.green = 0
theData.blue = 0

########################################################
#Callback functions for each subscriber
#	+ any other custom functions needed
#Do as little processing in the callback as possible
#Try to just store the incoming data + do all
#	processing in the main loop
########################################################
def updateLocalData(data):
    global colorImage, isColorImageReady, ratio
    colorImage = data
    isColorImageReady = True
	
def updateGlobalData(data):
    global colorImage, isColorImageReady, ratio
    colorImage = data
    isColorImageReady = True
	
def updateLocation(data):
    global colorImage, isColorImageReady, ratio
    colorImage = data
    isColorImageReady = True

def main():
    
	########################################################
	#Initialize the node, any subscribers and any publishers
	#TODO: Change data types of subscribers
	########################################################
    rospy.init_node('goal_pso_node', anonymous=True)
    rospy.Subscriber("/local_sensor_data", SensorData, updateLocalData, queue_size=10)
    rospy.Subscriber("/global_sensor_data", SensorData, updateGlobalData, queue_size=10)
    rospy.Subscriber("/location_data", Image, updateLocation, queue_size=10)
    pub = rospy.Publisher('suggested_movement', Image, queue_size=10)
	
	########################################################
	#Wait here for any data that needs to be ready
	#For data that would crash the program if it was not
	#	ready yet
	########################################################
	
    while not rospy.is_shutdown():

		
		########################################################
		#All code for processing data/algorithm goes here
		########################################################

        if(counter = 3) ##iteration biased to the current max

        else: ##iterations not biased
		
		########################################################
		#Publish data here
		########################################################
        pub.publish(imageMessage)
        



 
if __name__ == '__main__':
        main()
