#test rx_ucd_data, listen to data from the Py

import numpy as np
import json
import time
import serial
from rx_ucd_data import rx_ucd_data

#create serial object
ser = serial.Serial(
    port='/dev/serial0', #port on UP board, pins # 8 (TX) and 10 (RX). Don't forget to connect GND
    baudrate=9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
)

#listen to Py
while 1:
    #num = ser.in_waiting()
    if ser.inWaiting() > 14:
        message = ser.read(15)
        #if message: 
        print(message)  
        rx_ucd_data(message.decode('utf-8'))