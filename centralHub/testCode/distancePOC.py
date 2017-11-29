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

depth_width = 640 #320
depth_height = 480 #240
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
        
        #print("depth height: " + str(depthImage.height) + " depth width: " + str(depthImage.width))
        #print("image height: " + str(colorImage.height) + " image width: " + str(colorImage.width))
    
        centerDepth1 = depth.item(depth_height/2,depth_width/2)
        centerDepth2 = depth.item(depth_height/2+1,depth_width/2)
        centerDepth3 = depth.item(depth_height/2,depth_width/2+1)
        centerDepth4 = depth.item(depth_height/2,depth_width/2-1)
        centerDepth5 = depth.item(depth_height/2-1,depth_width/2)
        
        centerDepth = 0
        count = 0
        if not math.isnan(centerDepth1):
            centerDepth = centerDepth + centerDepth1
            count = count + 1
            
        if not math.isnan(centerDepth2):
            centerDepth = centerDepth + centerDepth2
            count = count + 1
            
        if not math.isnan(centerDepth3):
            centerDepth = centerDepth + centerDepth3
            count = count + 1
            
        if not math.isnan(centerDepth4):
            centerDepth = centerDepth + centerDepth4
            count = count + 1
            
        if not math.isnan(centerDepth5):
            centerDepth = centerDepth + centerDepth5
            count = count + 1
        
        if count > 0:
            centerDepth = centerDepth/count
        
        topWidth = 2*centerDepth*math.tan(math.radians(32.4)) #57 degrees 58.5 degrees
        sideHeight = 2*centerDepth*math.tan(math.radians(25.2)) #43 degrees 46.6 degrees 24.3 orig
        
        #print("center: " + str(centerDepth) + " top: " + str(topWidth) + " side: " + str(sideHeight))
        
        cv2.putText(color_image, str(topWidth), (im_width/2+15,0+25), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 2)
        cv2.putText(color_image, str(sideHeight), (0+25,im_height/2+10), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 2)
        cv2.putText(color_image, str(centerDepth), (im_width/2,im_height/2), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 2)

        imageMessage = bridge.cv2_to_imgmsg(color_image, "bgr8")
        pub.publish(imageMessage)
    cv2.destroyAllWindows()


 
if __name__ == '__main__':
    main()
