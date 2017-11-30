#!/usr/bin/env python
import roslib
#roslib.load_manifest('rosopencv')
import sys
import rospy
import cv2
import math
import imutils
from swarm.msg import RobotLocation, RobotLocationList
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

locationList = RobotLocationList()
locationList.robotList = []

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

bridge = CvBridge()

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
    global depthImage, isDepthImageReady, colorImage, isColorImageReady, bridge, dist_width, dist_height, image_width, image_height, locationList
    
    ########################################################
    #Initialize the node, any subscribers and any publishers
    ########################################################
    rospy.init_node('image_converter_node', anonymous=True)
    rospy.Subscriber("/camera/depth/image", Image, updateDepthImage, queue_size=10)
    rospy.Subscriber("/camera/rgb/image_color", Image, updateColorImage, queue_size=10)
    locPub = rospy.Publisher('robot_location', RobotLocationList, queue_size=10)
    imPub = rospy.Publisher('location_image', Image, queue_size=10)
    maskPub = rospy.Publisher('mask_image', Image, queue_size=10)
    
    while not isDepthImageReady:
        pass
    
    #bridge = CvBridge()
    
    ########################################################
    #Determine size of search area
    ########################################################
    calculateSearchAreaDistance()
    
    while not isColorImageReady:
        pass

    while not rospy.is_shutdown():
        #Reset list of robot locations
        locationList.robotList = []
        locationList.numRobots = 0
        locationList.width = image_width
        locationList.height = image_height
        
        try:
            color_image = bridge.imgmsg_to_cv2(colorImage, "bgr8")
            color_image_raw = color_image.copy()
        except CvBridgeError, e:
            print e
            print "colorImage"
        
        # Add distance calculations to color_image
        stringTopWidth = "%.2f" % dist_width
        stringSideHeight = "%.2f" % dist_height
        cv2.putText(color_image, stringTopWidth, (image_width/2+15,0+25), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 2)
        cv2.putText(color_image, stringSideHeight, (0+25,image_height/2+10), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 2)
        
        # Find triangles and identify robots
        gray = cv2.cvtColor(color_image_raw, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (11, 11), 0) #orig 5x5
        thresh = cv2.threshold(blur, 65, 255, cv2.THRESH_BINARY)[1]
        thresh = cv2.erode(thresh, None, iterations=2)
        thresh = cv2.dilate(thresh, None, iterations=2)
        
        maskPub.publish(bridge.cv2_to_imgmsg(thresh, "mono8"))
        #cv2.imshow("mask image", thresh)
        #cv2.waitKey(1)

        cntrs = cv2.findContours(thresh.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE) #orig RETR_EXTERNAL
        cntrs = cntrs[0] if imutils.is_cv2() else cntrs[1]
        #maxC = max(cntrs, key=cv2.contourArea)
        
        for c in cntrs:
            perim = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.04 * perim, True) #0rig 0.04*perim
            if len(approx) == 3:
                if not drawRobotInfo(color_image, color_image_raw, approx, c):
                    cv2.drawContours(color_image, [approx], -1, (0,255,0), 2)
            #else: #draw all contours for debugging purposes
                #cv2.drawContours(color_image, [approx], -1, (255,0,0), 2)

        imPub.publish(bridge.cv2_to_imgmsg(color_image, "bgr8"))
        locPub.publish(locationList)
    cv2.destroyAllWindows()

def drawRobotInfo(color_image, color_image_raw, pts, contour):
    global xPixelDist, yPixelDist, locationList
    
    #Get line distances - calculate tip point
    line1 = calculateLineDistance(pts[0][0], pts[1][0])
    line2 = calculateLineDistance(pts[0][0], pts[2][0])
    line3 = calculateLineDistance(pts[1][0], pts[2][0])
    
    #print("line 1: " + str(line1) + " line 2: " + str(line2) + " line 3: " + str(line3))
    
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
        
    #print("oline 1: " + str(oLine1) + " oline 2: " + str(oLine2))

    #Calculate triangle area
    area = oLine1 * oLine2 / 2
    lineDiff = 100*math.fabs(oLine1 - oLine2)/oLine2
    
    #print("\nArea: " + str(area) + " line diff: " + str(lineDiff))
    
    #check area + line distances to determine if a robot    
    # expected area = 55.125 cm^2
    maxArea = 70 #cm^2
    minArea = 20
    allowLineDiff = 20 #% difference allowed between 2 oLines - triangle should be isosceles
    
    if area >= minArea and area <= maxArea and lineDiff < allowLineDiff:
        robot = RobotLocation()
        
        #if a robot:
        #   calculate distance coordinates
        M = cv2.moments(contour)
        centerX_pix = int(M["m10"] / M["m00"]) 
        centerY_pix = int(M["m01"] / M["m00"])
        
        #Find color of triangle
        robotInfo = classify_triangle_color(color_image_raw, centerX_pix, centerY_pix)
        robot.robotID = robotInfo[1]
        robot.robotColor = robotInfo[0]

        #Calculate location
        centerX_dist = centerX_pix * yPixelDist
        centerY_dist = centerY_pix * xPixelDist
        robot.x = centerX_dist
        robot.y = centerY_dist
        
        #Calculate angle https://www.wikihow.com/Find-the-Angle-Between-Two-Vectors
        #length = math.sqrt((tipX-centerX_pix)**2 + (tipY-centerY_pix)**2)
        #dot = 3*(tipX-centerX_pix)
        #angle = math.degrees(math.acos(dot/(3*length)))
        #if tipY > centerY_pix:
        #    angle = -1*angle
        #robot.angle = angle
        #Check if on xy axis
        x = tipX - centerX_pix
        y = centerY_pix - tipY
        
        angle = 0
        if x >= 0 and y == 0:
            angle = 0
        elif x == 0 and y > 0:
            angle = 90
        elif x < 0 and y == 0:
            angle = 180
        elif x == 0 and y < 0:
            angle = 270
        elif x > 0 and y > 0: #quadrant I
            angle = math.degrees(math.atan(float(y)/float(x)))        
        elif x < 0 and y > 0: #quadrant II
            angle = 180 + math.degrees(math.atan(float(y)/float(x)))
        elif x < 0 and y < 0: #quadrant III
            angle = 180 + math.degrees(math.atan(float(y)/float(x)))
        else: # if x > 0 and y < 0: #quadrant IV
            angle = 360 + math.degrees(math.atan(float(y)/float(x)))
        
        robot.angle = angle
        
        #Draw bounding triangle + direction arrow
        cv2.drawContours(color_image, [pts], -1, (0, 0, 255), 2)
        cv2.arrowedLine(color_image, (centerX_pix, centerY_pix), (tipX, tipY), (255, 0, 0), 2)

        #Display location
        locationTextX = "%.2f" % centerX_dist
        locationTextY = "%.2f" % centerY_dist
        locationText = locationTextX + ", " + locationTextY
        angleStr = "%.2f" % angle
        angleText = angleStr + " deg"
        
        #print("robot at: " + locationText)
        
        cv2.putText(color_image, locationText, (centerX_pix,centerY_pix+75), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 2)
        cv2.putText(color_image, angleText, (centerX_pix,centerY_pix+105), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 2)
        cv2.putText(color_image, str(robot.robotID), (centerX_pix,centerY_pix+135), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 2)
        
        locationList.robotList.append(robot)
        locationList.numRobots = locationList.numRobots + 1
        return True
    else: 
        return False

#Calculate distance between points in cm
def calculateLineDistance(pt1, pt2):
    global xPixelDist, yPixelDist
    dist = math.sqrt(((pt1[0] - pt2[0])*xPixelDist*100)**2 + ((pt1[1] - pt2[1])*yPixelDist*100)**2)
    return dist

def calculateSearchAreaDistance():
    global depthImage, image_height, image_width, xPixelDist, yPixelDist, dist_width, dist_height, bridge
    
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
    dist_height = 2*centerDepth*math.tan(math.radians(25.2))
    
    xPixelDist = dist_width / image_width
    yPixelDist = dist_height / image_height    
    
# From: https://stackoverflow.com/questions/36439384/classifying-rgb-values-in-python
def classify_triangle_color(image, triangle_center_x, triangle_center_y):
    #Get an average rgb value of the triangle
    
    b1, g1, r1 = image[triangle_center_y, triangle_center_x]
    b2, g2, r2 = image[triangle_center_y+1, triangle_center_x]
    b3, g3, r3 = image[triangle_center_y-1, triangle_center_x]
    b4, g4, r4 = image[triangle_center_y, triangle_center_x+1]
    b5, g5, r5 = image[triangle_center_y, triangle_center_x-1]
    
    
    red = int((int(r1) + int(r2) + int(r3) + int(r4) + int(r5))/5)
    green = int((int(g1) + int(g2) + int(g3) + int(g4) + int(g5))/5)
    blue = int((int(b1) + int(b2) + int(b3) + int(b4) + int(b5))/5)
    
    rgb_tuple = (red, green, blue)
    
    #print(str(red) + " " + str(green) + " " + str(blue))
    #print("\n")
        
    # eg. rgb_tuple = (2,44,300)

    # add as many colors as appropriate here, but for
    # the stated use case you just want to see if your
    # pixel is 'more red' or 'more green'
    colors = {"red": (255, 0, 0),
              "green" : (0,255,0),
              "blue" : (0,0,255),
              "yellow" : (255,255,0),
              }
    ids = {"red" : 1,
              "green" : 2,
              "blue" : 3,
              "yellow" : 4,
              }

    manhattan = lambda x,y : abs(x[0] - y[0]) + abs(x[1] - y[1]) + abs(x[2] - y[2]) 
    distances = {k: manhattan(v, rgb_tuple) for k, v in colors.items()}
    color = min(distances, key=distances.get)
    robotID = ids.get(color)
    #print(color)
    return (color, robotID)
 
if __name__ == '__main__':
        main()
