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

def calculateLineDistance(pt1, pt2):
    dist = math.sqrt((pt1[0] - pt2[0])**2 + (pt1[1] - pt2[1])**2)
    return dist

def getTipPoint(pts):
    #print(str(pts))
    #Calculate line distances
    line1 = calculateLineDistance(pts[0][0], pts[1][0])
    line2 = calculateLineDistance(pts[0][0], pts[2][0])
    line3 = calculateLineDistance(pts[1][0], pts[2][0])
    
    #Find longest line - draw a circle in the 1 point not on that line
    if line1 > line2 and line1 > line3:
        #Line 1 max
        x = pts[2][0][0]
        y = pts[2][0][1]
    elif line2 > line3:
        #Line 2 max
        x = pts[1][0][0]
        y = pts[1][0][1]
    else:
        #Line 3 max
        x = pts[0][0][0]
        y = pts[0][0][1]
        
    return (x,y)


def updateColorImage(data):
    global colorImage, isColorImageReady, ratio
    colorImage = data
    isColorImageReady = True

def main():
    global colorImage, isColorImageReady, greenLower, greenUpper, boundsInit, bridge
    imPub = rospy.Publisher('location_image', Image, queue_size=10)
    maskPub = rospy.Publisher('mask_image', Image, queue_size=10)
    rospy.init_node('image_converter', anonymous=True)
    rospy.Subscriber("/camera/rgb/image_color", Image, updateColorImage, queue_size=10)
    bridge = CvBridge()
    #cv2.namedWindow("Color Image")
    #cv2.namedWindow("mask image")


    while not isColorImageReady:
        pass

    while not rospy.is_shutdown():
        try:
            color_image = bridge.imgmsg_to_cv2(colorImage, "bgr8")
        except CvBridgeError, e:
            print e
            print "colorImage"

        #resize_frame = imutils.resize(color_image, width=300)
        #ratio = color_image.shape[0] / float(resize_frame.shape[0])
        
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        #blur = cv2.GaussianBlur(gray, (5, 5), 0)
        thresh = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY)[1]
        thresh = cv2.erode(thresh, None, iterations=1)
        thresh = cv2.dilate(thresh, None, iterations=1)
        
        maskPub.publish(bridge.cv2_to_imgmsg(thresh, "mono8"))
        #cv2.imshow("mask image", thresh)
        #cv2.waitKey(1)

        cntrs = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cntrs = cntrs[0] if imutils.is_cv2() else cntrs[1]
        #maxC = max(cntrs, key=cv2.contourArea)
        
        for c in cntrs:
            perim = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.04 * perim, True)
            if len(approx) == 3:
                #c = c.astype("float")
                #c *= ratio
                #c = c.astype("int")
                #print(str(approx))
                M = cv2.moments(c)
                centerX = int(M["m10"] / M["m00"])
                centerY = int(M["m01"] / M["m00"])
                cv2.drawContours(color_image, [approx], -1, (0, 0, 255), 2)
                #cv2.circle(color_image, (centerX, centerY), 7, (255, 0, 0), -1)
                tip = getTipPoint(approx)
                cv2.arrowedLine(color_image, (centerX, centerY), tip, (255, 0, 0), 2)
                
        #cv2.rectangle(color_image, (xLocation-10,yLocation-10), (xLocation+10,yLocation+10), (0,255,0), 2)
        imageMessage = bridge.cv2_to_imgmsg(color_image, "bgr8")
        #try:
        #    conv_image = bridge.imgmsg_to_cv2(colorImage, "bgr8")
        #except CvBridgeError, e:
        #    print e
        #    print "colorImage"
        
        imPub.publish(imageMessage)
        #cv2.imshow("Color Image", conv_image)
        #cv2.waitKey(1)

    cv2.destroyAllWindows()


 
if __name__ == '__main__':
        main()
