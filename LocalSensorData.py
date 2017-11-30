#!/usr/bin/env python
import roslib
#roslib.load_manifest('rosopencv')
import sys
import rospy
import math
from std_msgs.msg import String
from sensor_msgs.msg import Image
from smbus2 import SMBus
import time
from swarm.msg import SensorData

def main():
    global data
    data = SensorData()
    sensorAddr =  0x44 ##address of the light sensor
    i2c = SMBus(1) ##initialize SMBus object using the bus number
    i2c.write_byte_data(0x44, 1, 0xD) ##configures the light sensor
    
    ########################################################
    #Initialize the node, any subscribers and any publishers
    #Note no subscribers needed for this node
    ########################################################
    rospy.init_node('local_sensor_data_node', anonymous=True)
    pub = rospy.Publisher('local_sensor_data', SensorData, queue_size=10)
    
    ########################################################
    #Wait here for any data that needs to be ready
    #For data that would crash the program if it was not
    #   ready yet
    ########################################################

    while not rospy.is_shutdown():
    ########################################################
    #All code for processing data/algorithm goes here
    ########################################################

        ##reads and compiles the green data
        greenLow = i2c.read_byte_data(0x44, 9)
        greenHigh = i2c.read_byte_data(0x44, 10)
        data.green = greenHigh<<8 | greenLow

        ##reads and compiles the red data
        redLow = i2c.read_byte_data(0x44, 11)
        redHigh = i2c.read_byte_data(0x44, 12)
        data.red = redHigh<<8 | redLow

        ##reads and compiles the blue data
        blueLow = i2c.read_byte_data(0x44, 13)
        blueHigh = i2c.read_byte_data(0x44, 14)
        data.blue = blueHigh<<8 | blueLow
        
        ########################################################
        #Publish data here
        #########################################################
        pub.publish(data)

        time.sleep(1)


 
if __name__ == '__main__':
        main()
