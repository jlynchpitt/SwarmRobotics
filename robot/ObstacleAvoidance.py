#!/usr/bin/env python
import roslib
#roslib.load_manifest('rosopencv')
import sys
import rospy
import math
from std_msgs.msg import String
from sensor_msgs.msg import Image

colorImage = Image()
isColorImageReady = False;

########################################################
#Callback functions for each subscriber
#	+ any other custom functions needed
#Do as little processing in the callback as possible
#Try to just store the incoming data + do all
#	processing in the main loop
########################################################
def updateSuggestedMovemement(data):
    global colorImage, isColorImageReady, ratio
    colorImage = data
    isColorImageReady = True
	
def updateLocation(data):
    global colorImage, isColorImageReady, ratio
    colorImage = data
    isColorImageReady = True

def main():
    global colorImage, isColorImageReady
    
	########################################################
	#Initialize the node, any subscribers and any publishers
	#TODO: Change data types of subscribers
	########################################################
    rospy.init_node('obstacle_avoidance_node', anonymous=True)
    rospy.Subscriber("/suggested_movement", Image, updateSuggestedMovement, queue_size=10)
    rospy.Subscriber("/location_data", Image, updateLocation, queue_size=10)
	pub = rospy.Publisher('commanded_movement', Image, queue_size=10)
	
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
        
		
		########################################################
		#Publish data here
		########################################################
        pub.publish(imageMessage)
        



 
if __name__ == '__main__':
        main()
