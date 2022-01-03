""" Main Flight Software Class
@author: Xiao-Bao Bao
@license: GNU Commons
@version 1.0

@brief Master Python source file (primary function file)

This is the master file for the main function of the InspectionCV primary payload
and performs the needed schedule function calls for the flight computer's data collection.
Primary functions include: 
1. Payload deployment
2. Invoking model inference results
3. Serial data packet comm to the S/C FC
"""
# =====================================
# ==         CONFIGURATION           ==
# =====================================

from os import *
from time import time
from loguru import logger

import numpy as np
import math
import loguru
import Burnwire as bw
import RPi.GPIO as GPIO
import serial
import threading

# =====================================
# ==         DEPLOYMENT VARS         ==
# =====================================
PHOTODIODE_PIN = 9
INPUT_PIN = 10
BURN_PIN_1 = 11
BURN_PIN_2 = 12
LED_PIN = 14
BURNWIRE_PINS = (BURN_PIN_1, BURN_PIN_2)
BURNWIRE_WAIT_TIME = 20 # mins
LUMINOSITY_THRESHOLD = 1

# =====================================
# ==       COMMS GLOBAL VARS         ==
# =====================================
BAUDRATE = 9600
SYS_TIMEOUT = 5
mini_UART = '/dev/ttyS0'
PL011 = '/dev/ttyAMA0'
SERIAL_PORT = PL011
ser=serial.Serial()

# =====================================
# ==        MISC GLOBAL VARS         ==
# =====================================
SCIENCE = False
MAX_TEMP = 85 # measured in degrees C (see datasheet -20C - +85C)


def startTimer():
    """ TODO: initializes the global timer var as a seperate thread process
    """
    pass

def checkDeployed():
    """ Determines if the payload mechanism has been triggered and deployed by
        comparing ambient light to some static threshold value
        
    Parameters
    ----------
    val: float
        Contains the digital photodiode readings
    
    Returns
    ---------
    boolean output based on if deploy criterion met
    """
    val = GPIO.input(PHOTODIODE_PIN)
    if val < LUMINOSITY_THRESHOLD:
        return False
    else
        return True

def initializeComputer():
    """ Setup calls to RPi.GPIO to initialize pin numbers to board specs and starts
        the experiment timer
    """
    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(False)
    GPIO.setup(LED_PIN, GPIO.OUT)
    startTimer()
        
def initializeBurnwire():
    """ Setup calls to RPi.GPIO to initialize pin numbers to board specs and assigns
        photodiode to a pulldown resistor
    """
    GPIO.setup(BURN_PIN_1, GPIO.OUT, initial = GPIO.LOW)
    GPIO.setup(BURN_PIN_2, GPIO.OUT, initial = GPIO.LOW)
    GPIO.setup(INPUT_PIN, GPIO.IN)
    GPIO.setup(PHOTODIODE_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.cleanup()
    
def fireBurnwire():
    """ Calls GPIO pins in order to run current through burn resistor and
        trigger the release mechanism
        
    Parameters
    ----------
    val: float
        Contains the digital photodiode readings
    
    Returns
    ---------
    
    """
    # Immediately perform burnwire deploy for 5 seconds
    GPIO.output(burnChanList, GPIO.HIGH)
    prtStatus = GPIO.output(burnChanList, not GPIO.input(burnChanList))
    time.sleep(5) # Wait 5 seconds
    # Shutdown burnwire!
    GPIO.output(burnChanList, GPIO.LOW)
    prtStatus += GPIO.output(burnChanList, not GPIO.input(burnChanList)) # Report to sys diagnostic
    time.sleep(10) # Wait 10 seconds

def serialSetup():
    SCIENCE = True
    # Initialize Serial Port
    ser = serial.Serial(port=SERIAL_PORT, baudrate=BAUDRATE,parity=Serial.PARITY_EVEN,
                            timeout=SYS_TIMEOUT,stopbits=serial.STOPBITS_ONE,
                            bytesize=serial.EIGHTBITS)
    counter = 0
    print(ser.name)
    ser.write(b'CV_Payload_Active')
    ser.open()
    

def verifySystem():
    # TODO: Perform RAM check, VRAM check, Power check, Lighting check, Camera check
    if !ser.is_OPEN:
        return False
    elif !checkTempCPU():
        return False
    pass

def checkTempCPU():
    result = system('vcgencmd measure_temp')
    temperature = float(result[5:9])
    print('Current CPU Temp is :', temperature)
    if temperature >= MAX_TEMP:
        print('Exceeding maximum allowable compute temp')
        return False
    else
        return True

def reboot():
    system('sudo restart')
    pass
    
"""
@precondition: results are of type bytes or bytearray
ex: 'hello'.encode('utf-8')
"""
def writePayloadData(results):
    while True:
        try:
            ser.write(b'Writing CV results: \n'%(results))
        except (serial.SerialTimeoutException):
            print("Serial Timed out! Re-attempting connection...")
            systemSetup()
        print(ser.in_waiting())
        print("Packet sent")
        time.sleep(1) # Wait one second for packet to send
        counter += 1
    ser.flush()

def setup():
    initializeComputer()
    serialSetup()
    initializeBurnwire()
    
    while !checkDeployed():
            prtStatus += "Burnwire not deployed, trying again"
            fireBurnwire()
            # If not deployed, wait 20 minutes
            if !checkDeployed():
                time.sleep(BURNWIRE_WAIT_TIME*1000)
                
    if !verifySystem():
        reboot()
    else:
        pass

def main():
    setup()
    while True:
        results = doScience()
        writePayloadData(results)
        #TODO: must be able to read from Rx and implement parity bit outcome
        pass
    ser.__exit__()    

if __name__ == "__main__":
    main()