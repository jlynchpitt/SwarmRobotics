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
from copy import deepcopy
import imutils

colorImage = Image()
isColorImageReady = False;
greenLower  = (29, 86, 6)# rgb values 
greenUpper = (64, 255, 255)
boundsInit = True
bridge = ""
ratio = 1


def mouseClick(event, x, y, flags, param):
    global greenLower, greenUpper, boundsInit, colorImage, isColorImageReady, bridge
    if event == cv2.EVENT_LBUTTONDOWN:
        """if isColorImageReady:
            pixel = colorImage[x,y]
            if boundsInit:
                if pixel[0] < greenLower[0]:
                    greenLower[0] = pixel[0]
                    
                if pixel[1] < greenLower[1]:
                    greenLower[1] = pixel[1]

                if pixel[2] < greenLower[2]:
                    greenLower[2] = pixel[2]

                #Check upper bounds
                if pixel[0] > greenUpper[0]:
                    greenUpper[0] = pixel[0]
                if pixel[1] > greenUpper[1]:
                    greenUpper[1] = pixel[1]
                if pixel[2] > greenUpper[2]:
                    greenUpper[2] = pixel[2]
            else:
                greenLower = deepcopy(pixel)
                greenUpper = deepcopy(pixel)
                boundsInit = True"""
        print("lower: " + str(greenLower) + " upper: " + str(greenUpper))
    
def updateColorImage(data):
    global colorImage, isColorImageReady, ratio
    try:
            frame = bridge.imgmsg_to_cv2(data, "bgr8")
            resize_frame = imutils.resize(frame, width=600)
            blur = cv2.GaussianBlur(resize_frame, (11, 11), 0)
            colorImage = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
            isColorImageReady = True
            ratio = frame.shape[0] / float(resize_frame.shape[0])
    except CvBridgeError, e:
        print e
        print "colorImage"

def main():
    global colorImage, isColorImageReady, greenLower, greenUpper, boundsInit, bridge
    rospy.init_node('image_converter', anonymous=True)
    rospy.Subscriber("/camera/rgb/image_color", Image, updateColorImage, queue_size=10)
    bridge = CvBridge()
    cv2.namedWindow("Color Image")
    cv2.namedWindow("mask image")
    cv2.setMouseCallback("Color Image", mouseClick)

    while not isColorImageReady:
        pass

    while not rospy.is_shutdown():
        #try:
        #    color_image = bridge.imgmsg_to_cv2(colorImage, "bgr8")
        #except CvBridgeError, e:
        #    print e
        #    print "colorImage"
        color_image = colorImage
        #construct mask of green objects
        if boundsInit:
            mask = cv2.inRange(color_image, greenLower, greenUpper)
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=1)
            
            cntrs = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cntrs = cntrs[0] if imutils.is_cv2() else cntrs[1]
            
            for c in cntrs:
                perim = cv2.arcLength(c, True)
                approx = cv2.approxPolyDP(c, 0.04 * perim, True)
                if len(approx) == 3:
                    print("Triangle found")
                    c = c.astype("float")
                    c *= ratio
                    c = c.astype("int")
                    cv2.drawContours(color_image, [c], -1, (0, 255, 0), 2)

            cv2.imshow("mask image", mask)
        

        #cv2.rectangle(color_image, (xLocation-10,yLocation-10), (xLocation+10,yLocation+10), (0,255,0), 2)
        cv2.imshow("Color Image", color_image)
        cv2.waitKey(1)

    cv2.destroyAllWindows()


 
if __name__ == '__main__':
    main()
