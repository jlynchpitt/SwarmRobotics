#!/usr/bin/env python
import roslib
#roslib.load_manifest('rosopencv')
import sys
import rospy
import cv2
import math
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

depthImage = Image()
isDepthImageReady = False;
colorImage = Image()
isColorImageReady = False;

depth_width = 320
depth_height = 240
im_width = 640
im_height = 480

def updateDepthImage(data):
    global depthImage, isDepthImageReady
    depthImage = data
    isDepthImageReady = True

def updateColorImage(data):
    global colorImage, isColorImageReady
    colorImage = data
    isColorImageReady = True

def main():
    global depthImage, isDepthImageReady, colorImage, isColorImageReady, depth_width, depth_height, im_width, im_height
    rospy.init_node('image_converter', anonymous=True)
    rospy.Subscriber("/camera/depth/image", Image, updateDepthImage, queue_size=10)
    rospy.Subscriber("/camera/rgb/image_color", Image, updateColorImage, queue_size=10)
	pub = rospy.Publisher('distance_image', Image, queue_size=10)
    bridge = CvBridge()

    while not isDepthImageReady or not isColorImageReady:
        pass

    while not rospy.is_shutdown():
        try:
            depth = bridge.imgmsg_to_cv2(depthImage, desired_encoding="passthrough")
        except CvBridgeError, e:
            print e
            print "depthImage"

        try:
            color_image = bridge.imgmsg_to_cv2(colorImage, "bgr8")
        except CvBridgeError, e:
            print e
            print "colorImage"
        
		print("depth height: " + str(depth.height) + " depth width: " str(depth.width))
		print("image height: " + str(color_image.height) + " image width: " str(color_image.width))
	
        centerDepth = depth.item(depth_height/2,depth_width/2,0)
        topDepth = depth.item(0,depth_width/2,0)
        sideDepth = depth.item(depth_height/2,0,0)
		
		#halfSideLength^2 + centerDepth^2 = topDepth^2
		sideLength = 2*sqrt(math.pow(topDepth, 2) - math.pow(centerDepth, 2))
        sideLengthStr = "%.2f" % sideLength

		#halfTopLength^2 + centerDepth^2 = sideDepth^2
		topLength = 2*sqrt(math.pow(sideDepth, 2) - math.pow(centerDepth, 2))
        topLengthStr = "%.2f" % topLength

        cv2.putText(color_image, topLengthStr, (im_width/2+15,0+10), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 2)
        #cv2.putText(color_image, sideLengthStr, (0+15,im_height/2+10), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 2)

		imageMessage = bridge.cv2_to_imgmsg(color_image, "bgr8")
        imPub.publish(imageMessage)
    cv2.destroyAllWindows()


 
if __name__ == '__main__':
    main()
