#!/usr/bin/env python
import roslib
#roslib.load_manifest('rosopencv')
import sys
import rospy
import cv2
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
def updateColorImage(data):
    global colorImage, isColorImageReady, ratio
    colorImage = data
    isColorImageReady = True

def main():
    global colorImage, isColorImageReady
    
	########################################################
	#Initialize the node, any subscribers and any publishers
	########################################################
    rospy.init_node('image_converter', anonymous=True)
    rospy.Subscriber("/camera/rgb/image_color", Image, updateColorImage, queue_size=10)
	imPub = rospy.Publisher('location_image', Image, queue_size=10)
    maskPub = rospy.Publisher('mask_image', Image, queue_size=10)
	
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
        imPub.publish(imageMessage)
        

    cv2.destroyAllWindows()


 
if __name__ == '__main__':
        main()
