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
from loguru import RotationFunction, logger
from pathlib import Path, WindowsPath

import numpy as np
import math
import loguru
#import Burnwire as bw
import RPi.GPIO as GPIO
import serial
import threading

import DetectronPredictor
import YoloPredictor

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
ser = serial.Serial()

# =====================================
# ==           CV VARS               ==
# =====================================
FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]  # YOLOv5 root directory

Detectron2_WEIGHTS = ROOT / 'detmodel.pt'
Detectron2_SOURCE = ROOT / 'data/images'
YOLO_WEIGHTS = ROOT / 'yolov5s.pt'
YOLO_SOURCE = ROOT / 'data/images'

EXPOSURE_TIME = 1 # ms
DELAY = 1 # ms
WIDTH = 800 # px
HEIGHT = 600 # px

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
    else:
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
    pass
    """
    # Immediately perform burnwire deploy for 5 seconds
    GPIO.output(BURNWIRE_PINS, GPIO.HIGH)
    prtStatus = GPIO.output(BURNWIRE_PINS, not GPIO.input(BURNWIRE_PINS))
    time.sleep(5) # Wait 5 seconds
    # Shutdown burnwire!
    GPIO.output(BURNWIRE_PINS, GPIO.LOW)
    prtStatus += GPIO.output(BURNWIRE_PINS, not GPIO.input(BURNWIRE_PINS)) # Report to sys diagnostic
    time.sleep(10) # Wait 10 seconds
    pass

def camLightOn():
    GPIO.output(LED_PIN, GPIO.HIGH)
    pass

def camLightOff():
    GPIO.output(LED_PIN, GPIO.LOW)
    pass

def serialSetup():
    """ 
        
    Parameters
    ----------
    val: 
    
    Returns
    ---------
    
    """
    SCIENCE = True
    # Initialize Serial Port
    ser = serial.Serial(port=SERIAL_PORT, baudrate=BAUDRATE,
                        parity=serial.PARITY_ODD,timeout=SYS_TIMEOUT,
                        stopbits=serial.STOPBITS_ONE,bytesize=serial.EIGHTBITS)
    counter = 0
    print(ser.name)
    ser.write(b'CV_Payload_Active')
    ser.open()
    

def verifySystem(serial):
    """ Performs system diagnotic checks on power, temperature, lighting, and comms

    Arguments
    ----------
    serial: Serial
        Serial object returns the status of the serial connection to the S/C FC, can ping
        for timeout and data packet state
        
    
    Returns
    ---------
    False if any system related check function either also returns false or times out
    True if all systems check out to true
    """
    # TODO: Perform RAM check, VRAM check, Power check, Lighting check, Camera check
    if not serial.is_open:
        return False
    elif not checkTempCPU():
        return False
    else:
        return True

def checkTempCPU():
    """ Checks the SoC temperature in the core processor
        
    Parameters
    ----------
    result: string
        Shell command to pull measured processor temperature
    temperature: float
        CPU processing temperature in degrees Celsius by pulling a substring of output
    
    Returns
    ---------
    True of the processor is within a temperature range set by MAX_TEMP
    False if the processor temperature exceeds allowable (system will auto throttle)
    """
    result = system('vcgencmd measure_temp')
    temperature = float(result[5:9])
    print('Current CPU Temp is :', temperature)
    if temperature >= MAX_TEMP:
        print('Exceeding maximum allowable compute temp')
        return False
    else:
        return True

def reboot():
    """ Shell command to restart the RPi
    """
    system('sudo restart')
    pass

def takePicture(exposureTime, delay, width, height):
    """TODO: Perform cam connection check, lighting check"""
    system('libcamera-jpeg -o handrail-input.jpg -t 5000 --width 800 --height 600')
    pass


def performInference(model, source):
    """ Calls CV models one after another, ensuring that light and dark lighting
        conditons are also created for each CV model
    TODO: Need to determine fault handling for if a model fails to perform successful inference
          i.e. memory leaks or failed inference.
    Arguments
    ---------
    model: string 
        input argument that tells function which CV model to invoke
    source: string
        File path to the image that we want to perform inference on

    Returns
    --------
    resultsDF: string
        Filepath to dataframe of output results
    """
    camLightOn()
    DetectronResultsLit = DetectronPredictor.detect(source)
    YoloResultsLit = YoloPredictor

    camLightOff()
    DetectronResultsDark = DetectronPredictor.detect(source)
    YoloResultsDark = YoloPredictor

    return [{'Light': (DetectronResultsLit, YoloResultsLit)} ,
            {'Dark':(DetectronResultsDark, YoloResultsDark)}]
    
    
def writePayloadData(results):
    """ Takes CV inference results from the model and writes the data to the S/C FC
        via UART serial protocol

    Arguments
    ----------
    results: string
        A properly formatted dataframe in the format of a string
        TODO: results is probably a dataframe and must be converted into a string and
        then converted into bytes for the input buffer
        
    Parameters
    ----------
    bytesInBuffer: int
        Number of bytes still in the input buffer
    counter: int
        TODO: implement better timer var to track payload write process
    packetStatus: bool
        Carries the status of the write process
    
    Exceptions
    ---------
    SerialTimeoutException: Exception raised by Serial object if the connection
    times out, if so, restart the system serial connection.

    Returns
    ---------
    True if the data packet was sent without any isses
    False if otherwise
    """
    packetStatus = False
    counter = 0
    while True:
        try:
            ser.write(b'Writing CV results: \n'%(results))
        except (serial.SerialTimeoutException):
            # Reset the buffer and reset the serial conection
            print("Serial Timed out! Re-attempting connection...")
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            serialSetup()

        bytesInBuffer = ser.in_waiting()
        print(bytesInBuffer)
        print(ser.out_waiting())
        print("Packet sent")
        time.sleep(1) # Wait one second for packet to send
        counter += 1

        if(bytesInBuffer is None):
            packetStatus = True
            break

    ser.flush()
    return packetStatus

def setup():
    """ Performs initial bootup sequence once and deploys the payload mechanism
        
    Parameters
    ----------
    val: 
    """
    initializeComputer()
    serialSetup()
    initializeBurnwire()
    
    while not checkDeployed():
            prtStatus += "Burnwire not deployed, trying again"
            fireBurnwire()
            # If not deployed, wait 20 minutes
            if not checkDeployed():
                time.sleep(BURNWIRE_WAIT_TIME*1000)
                
    if not verifySystem():
        reboot()
    else:
        pass

def main():
    """ Main loop
    """
    setup()

    print(timer)

    while timer % timeInterval != 0:
        """ Performs an inference every six minutes """
        imgPath = takePicture(EXPOSURE_TIME, DELAY, WIDTH, HEIGHT)
        results = performInference(imgPath)
        writePayloadData(results)
        #TODO: must be able to read from Rx and implement parity bit outcome
        pass
    ser.__exit__()    

if __name__ == "__main__":
    main()