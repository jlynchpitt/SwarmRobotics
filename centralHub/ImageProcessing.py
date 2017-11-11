#!/usr/bin/env python
import roslib
#roslib.load_manifest('rosopencv')
import sys
import rospy
import cv2
import math
from std_msgs.msg import String
from sensor_msgs.msg import Image

depthImage = Image()
isDepthImageReady = False;
colorImage = Image()
isColorImageReady = False;

dist_width = 0
dist_height = 0

image_width = 640
image_height = 480

xPixelDist = 0
yPixelDist = 0

########################################################
#Callback functions for each subscriber
#   + any other custom functions needed
#Do as little processing in the callback as possible
#Try to just store the incoming data + do all
#   processing in the main loop
########################################################
def updateDepthImage(data):
    global depthImage, isDepthImageReady
    depthImage = data
    isDepthImageReady = True
    
def updateColorImage(data):
    global colorImage, isColorImageReady, ratio
    colorImage = data
    isColorImageReady = True

def main():
    global depthImage, isDepthImageReady, colorImage, isColorImageReady
    
    ########################################################
    #Initialize the node, any subscribers and any publishers
    ########################################################
    rospy.init_node('image_converter_node', anonymous=True)
    rospy.Subscriber("/camera/depth/image", Image, updateDepthImage, queue_size=10)
    rospy.Subscriber("/camera/rgb/image_color", Image, updateColorImage, queue_size=10)
    imPub = rospy.Publisher('location_image', Image, queue_size=10)
    maskPub = rospy.Publisher('mask_image', Image, queue_size=10)
    
    while not isDepthImageReady:
        pass
    
    ########################################################
    #Determine size of search area
    ########################################################
    calculateSearchAreaDistance()
    
    while not isColorImageReady:
        pass

    while not rospy.is_shutdown():
        try:
            color_image = bridge.imgmsg_to_cv2(colorImage, "bgr8")
        except CvBridgeError, e:
            print e
            print "colorImage"
        
        # Add distance calculations to color_image
        stringTopWidth = "%.2f" % topWidth
        stringSideHeight = "%.2f" % sideHeight
        cv2.putText(color_image, stringTopWidth, (im_width/2+15,0+25), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 2)
        cv2.putText(color_image, stringSideHeight, (0+25,im_height/2+10), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 2)
        
        # Find triangles and identify robots
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        thresh = cv2.threshold(blur, 150, 255, cv2.THRESH_BINARY)[1]
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
                if not drawRobotInfo(approx):
                    cv2.drawContours(color_image, [approx], -1, (0,255,0), 2)

        imPub.publish(bridge.cv2_to_imgmsg(color_image, "bgr8"))
    cv2.destroyAllWindows()

def drawRobotInfo(color_image, triPts, contour):
    global xPixelDist, yPixelDist
    
    #Get line distances - calculate tip point
    line1 = calculateLineDistance(pts[0][0], pts[1][0])
    line2 = calculateLineDistance(pts[0][0], pts[2][0])
    line3 = calculateLineDistance(pts[1][0], pts[2][0])
    
    #Find longest line
    if line1 > line2 and line1 > line3:
        #Line 1 max
        tipX = pts[2][0][0]
        tipY = pts[2][0][1]
        
        #longLine = line1
        oLine1 = line2
        oLine2 = line3
    elif line2 > line3:
        #Line 2 max
        tipX = pts[1][0][0]
        tipY = pts[1][0][1]
        
        #longLine = line2
        oLine1 = line1
        oLine2 = line3
    else:
        #Line 3 max
        tipX = pts[0][0][0]
        tipY = pts[0][0][1]
        
        #longLine = line3
        oLine1 = line2
        oLine2 = line1
        
    #Calculate triangle area
    area = oLine1 * oLine2 / 2
    lineDiff = 100*math.abs(oLine1 - oLine2)/oLine2
    
    #check area + line distances to determine if a robot    
    # expected area = 55.125 cm^2
    maxArea = 60 #cm^2
    minArea = 50 
    allowLineDiff = 5 #% difference allowed between 2 oLines - triangle should be isosceles
    
    if area >= minArea and area <= maxArea and lineDiffPer < allowLineDiff:
        #if a robot:
        #   calculate distance coordinates
        M = cv2.moments(contour)
        centerX_pix = int(M["m10"] / M["m00"]) 
        centerY_pix = int(M["m01"] / M["m00"])
        
        centerX_dist = centerX_pix * xPixelDist
        centerY_dist = centerY_pix * yPixelDist
        
        #Draw bounding triangle + direction arrow
        cv2.drawContours(color_image, [triPts], -1, (0, 0, 255), 2)
        cv2.arrowedLine(color_image, (centerX_pix, centerY_pix), (tipX, tipY), (255, 0, 0), 2)

        #Display location
        locationTextX = "%.2f" % centerX_dist
        locationTextY = "%.2f" % centerY_dist
        locationText = locationTextX + ", " + locationTextY
        
        cv2.putText(color_image, locationText, (centerX_pix,centerY_pix+75), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 2)
        return True
    else: 
        return False

#Calculate distance between points in cm
def calculateLineDistance(pt1, pt2):
    global xPixelDist, yPixelDist
    dist = math.sqrt(((pt1[0] - pt2[0])*xPixelDist)**2 + ((pt1[1] - pt2[1])*yPixelDist)**2)
    return dist

def calculateSearchAreaDistance():
    global depthImage, image_height, image_width, xPixelDist, yPixelDist, dist_width, dist_height
    
    try:
        depth = bridge.imgmsg_to_cv2(depthImage, desired_encoding="passthrough")
    except CvBridgeError, e:
        print e
        print "depthImage"

    centerDepth1 = depth.item(image_height/2, image_width/2)
    centerDepth2 = depth.item(image_height/2+1, image_width/2)
    centerDepth3 = depth.item(image_height/2, image_width/2+1)
    centerDepth4 = depth.item(image_height/2, image_width/2-1)
    centerDepth5 = depth.item(image_height/2-1, image_width/2)
    
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
    
    dist_width = 2*centerDepth*math.tan(math.radians(32.4))
    dist_height = 2*centerDepth*math.tan(math.radians(24.3))
    
    xPixelDist = dist_width / image_width
    yPixelDist = dist_height / image_height    
 
if __name__ == '__main__':
        main()
