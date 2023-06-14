#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#test rx_ucd_data, listen to data from the Py

import numpy as np
import json
import time
import mraa
import serial
from rx_ucd_data import rx_ucd_data
from read_data_string import read_data_string
from char2bits import char2bits

def check_input(inp):
	if inp[0] != '(' and inp[-1] != ')':
		print("start or end char broken:")
		#print(inp[0])
		return ""
	inp = inp[1:]
	inp = inp[:-1]
	new_string = inp.replace('!','')
	new_string = new_string.replace('?','')
	if new_string.isalnum() == 1:
		while len(inp) < 15:
			inp += '#'
		return inp
	return ""

#create serial object
ser = serial.Serial(
	port='/dev/ttyS4', #port on RPi, pins # 8 (TX) and 10 (RX). Don't forget to connect GND
	baudrate=19200, #115200, 19200
	parity=serial.PARITY_NONE,
	stopbits=serial.STOPBITS_ONE,
	bytesize=serial.EIGHTBITS,
	timeout=1
)

# Setup GPIO Pin
gpio_UCD_activate1 = mraa.Gpio(11)  #enable on GPIO 17
gpio_UCD_activate1.dir(mraa.DIR_OUT)
gpio_UCD_activate1.write(0)

gpio_UCD_activate2 = mraa.Gpio(13)  #start flag on GPIO 27
gpio_UCD_activate2.dir(mraa.DIR_OUT)
gpio_UCD_activate2.write(0)

#Send activation signal
print("Sending enable signal in 3 seconds")
time.sleep(3)
gpio_UCD_activate1.write(1)
print('GPIO 17 HIGH')

print("Sending start signal in 3 seconds")
time.sleep(3)
gpio_UCD_activate2.write(1)
print('GPIO 27 HIGH')

time.sleep(0.5)
print('Listening on UART Line')
#listen to Py
while 1:
	#str = ser.read_until(')',17)
	str_inp = ser.readline().strip().decode('UTF-8')
	if str_inp:
		print(str_inp)
		str_inp = check_input(str_inp)
		print(str_inp)
		if str_inp == "":
			continue;
		print(char2bits(str_inp)) # remove
		data = read_data_string(char2bits(str_inp))
		
		# display incoming data
		rx_ucd_data(str_inp)
		if data[0]:
			print("Packet type: HIO")
			print("Photo number: "+str(data[1]))
			print("YOLO detected? "+str(data[2]))
			print("MASK detected? "+str(data[3]))
			print("YBB X1: "+str(data[4]))
			print("YBB Y1: "+str(data[5]))
			print("YBB X2: "+str(data[6]))
			print("YBB Y2: "+str(data[7]))
			print("MBB X1: "+str(data[8]))
			print("MBB Y1: "+str(data[9]))
			print("MBB X2: "+str(data[10]))
			print("MBB Y2: "+str(data[11]))
			print("YOLO confidence: "+str(data[12]))
			print("MASK confidence: "+str(data[13]))
			print("Q30: "+str(data[14]))
			print("Q50: "+str(data[15]))
			print("Q70: "+str(data[16]))
			print("Q90: "+str(data[17]))
			print("Red: "+str(data[18]))
			print("Green: "+str(data[19]))
			print("Blue: "+str(data[20]))
			print("Temperature: "+str(data[21]))
		else:
			if data[1] %2 == 0:
				print("Packet type: HDD mag")
				print("Exp #: "+str(data[1]))
				print("BXA: "+str(data[2]))
				print("BXB: "+str(data[3]))
				print("BXC: "+str(data[4]))
				print("BYA: "+str(data[5]))
				print("BYB: "+str(data[6]))
				print("BYC: "+str(data[7]))
				print("BZA: "+str(data[8]))
				print("BZB: "+str(data[9]))
				print("BZC: "+str(data[10]))
				print("CD: "+str(data[11]))
				print("Temp: "+str(data[12]))
			else:
				print("Packet type: HDD acc")
				print("Exp #: "+str(data[1]))
				print("WXA: "+str(data[2]))
				print("WXB: "+str(data[3]))
				print("WXC: "+str(data[4]))
				print("WYA: "+str(data[5]))
				print("WYB: "+str(data[6]))
				print("WYC: "+str(data[7]))
				print("WZA: "+str(data[8]))
				print("WZB: "+str(data[9]))
				print("WZC: "+str(data[10]))
				print("CD: "+str(data[11]))
				print("Temp: "+str(data[12]))

    
#RSA


