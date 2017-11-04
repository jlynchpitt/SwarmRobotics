from smbus2 import SMBus
import time
from sys import getsizeof

## address of the device from i2cdetect
sensorAddr =  0x48

## initialize smbus object using bus number
i2c = SMBus(1)

#i2c.write_byte_data(sensorAddr, 1, 0xD)


#print ("Device ID: " + str(i2c.read_byte_data(0x44,0)))
num = 200
tempCount = 0
while(tempCount < num):
	#i2c.write_byte(sensorAddr, 0)
	temp = i2c.read_word_data(sensorAddr, 0)
	
	byte1 = temp & 0xFF
	byte2 = (temp & 0xFF00) >> 12
	
	temperature = ((byte1 << 4) + byte2)*0.0625

	print("temperature: " + str(temperature))
	tempCount = tempCount +1
	time.sleep(0.5)

