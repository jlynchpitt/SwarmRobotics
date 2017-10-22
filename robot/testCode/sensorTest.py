from smbus2 import SMBus
import time
from sys import getsizeof

## address of the device from i2cdetect
sensorAddr =  0x44

## initialize smbus object using bus number
i2c = SMBus(1)

i2c.write_byte_data(0x44, 1, 0xD)


print ("Device ID: " + str(i2c.read_byte_data(0x44,0)))
num = 100
temp = 0
while(temp < num):
	redLow = i2c.read_byte_data(0x44, 11)
	redHigh = i2c.read_byte_data(0x44, 12)

	##print("redLow: " +str(i2c.read_byte_data(0x44, 11)))
	##print("redHigh: "+str(i2c.read_byte_data(0x44, 12)))

	print("redHigh2: "+str(redHigh))
	red = redHigh<<8 | redLow

	print("Red: " +str(red))
	temp = temp +1
	time.sleep(1)

