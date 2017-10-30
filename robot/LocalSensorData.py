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

def main():
    global colorImage, isColorImageReady
    
	########################################################
	#Initialize the node, any subscribers and any publishers
	#Note no subscribers needed for this node
	########################################################
    rospy.init_node('local_sensor_data_node', anonymous=True)
	pub = rospy.Publisher('local_sensor_data', Image, queue_size=10)
	
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
