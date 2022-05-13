#test rx_ucd_data, send data from the Py

import numpy as np
import json
import time
import serial
from rx_ucd_data import rx_ucd_data

#create serial object
ser = serial.Serial(
    port='/dev/ttyAMA0',  #port on Py board, pins # 8 (TX) and 10 (RX)
    baudrate=115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
)

#send data to UP
data     = ["aaaaaaaaaaaaaaa"]
data.append("bbbbbbbbbbbbbbb")
data.append("ccccccccccccccc")
data.append("ddddddddddddddd")
data.append("eeeeeeeeeeeeeee")
data.append("fffffffffffffff")
data.append("ggggggggggggggg")
data.append("hhhhhhhhhhhhhhh")
data.append("iiiiiiiiiiiiiii")
data.append("jjjjjjjjjjjjjjj")
data.append("kkkkkkkkkkkkkkk")
data.append("lllllllllllllll")
data.append("mmmmmmmmmmmmmmm")
data.append("nnnnnnnnnnnnnnn")
data.append("ooooooooooooooo")
data.append("ppppppppppppppp")

for d in data:
        #message = d
        ser.write(d.encode('utf-8'))
        #if message: 
        print(d)  
        #rx_ucd_data(message)
        time.sleep(5)