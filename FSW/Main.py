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

import os
from os import *
from time import time
import sys
from loguru import RotationFunction, logger
from pathlib import Path, WindowsPath5

import numpy as np
import math
#import loguru
import RPi.GPIO as GPIO
import serial
import threading
import json

from serial.serialutil import SerialException

from Burnwire import Burnwire
from DriverLED import DriverLED
from Camera import Camera

sys.path.insert(0, os.getcwd() + '/FSW')

import Timer
#from DetectronPredictor import *
#from YoloPredictor import run



# =====================================
# ==         DEPLOYMENT VARS         ==
# =====================================
PD_POS = 9 # Photodiode GPIO20
#INPUT_PIN = 10
#BURN_PIN_1 = 11
#BURN_PIN_2 = 12
#LED_PIN = 14
#BURNWIRE_PINS = (BURN_PIN_1, BURN_PIN_2)
BURNWIRE_WAIT_TIME = 20 # mins
PHOTODIODE_THRESH = 1 # We need to test photodiode to find a good value for this

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
current_time = 0
parameterDB_path_name = '\home\pi\Desktop\ParameterDB'
state_variables_path = '\home\pi\Desktop\STATE_VARIABLES.json'
init_file = ""
deployed = False
burnwireFired = False


def startTimer(timer):
    """ TODO: initializes the global timer var as a seperate thread process
    """
    timer.start()

def clamp(n, minn, maxn):
    if n < minn:
        return minn
    elif n > maxn:
        return maxn
    else:
        return n

# TODO Write function that checks the photodiode, return True if above brightness
# threshold, return False otherwise

def checkPhotodiode(val):
    """
    Parameters
    ----------
    val : float 
        Value provided by GPIO pin attached to photodiode
    """
    if val is not type(float):
        if val > 1024 or val < 0:
            val = clamp(val, 0, 1024)

        if val > PHOTODIODE_THRESH:
            return True
    return False


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
    val = GPIO.input(PD_POS)
    if checkPhotodiode(val):
        return False
    else:
        return True

def initializeComputer():
    """ Setup calls to RPi.GPIO to initialize pin numbers to board specs and starts
        the experiment timer
    """
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    #GPIO.setup(LED_PIN, GPIO.OUT)
    startTimer()

def serialSetup():
    """ 
    Parameters
    ----------
    ser: Serial
        Serial object handles UART comms protocol with S/C
    
    Return
    ---------
        Serial object if successful connection and setup established, 
        otherwise, return None
    """
    #SCIENCE = True
    # Initialize Serial Port
    try:
        ser = serial.Serial(port=SERIAL_PORT, baudrate=BAUDRATE,
                            parity=serial.PARITY_ODD,timeout=SYS_TIMEOUT,
                            stopbits=serial.STOPBITS_ONE,bytesize=serial.EIGHTBITS)
    except SerialException as e:
        print("Issue with setting up UART connection!")
        print(e)
        return None
    #counter = 0
    print(ser.name)
    ser.write(b'CV_Payload_Active')
    ser.open()
    return ser

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

# DEPRECATED FUNCTION: 
# def takePicture(exposureTime, delay, width, height):
#     """TODO: Perform cam connection check, lighting check"""
#     system('libcamera-jpeg -o handrail-input.jpg -t 5000 --width 800 --height 600')
#     pass

def runInference(model, source):
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
    detectron_results_lit = DetectronPredictor.detect(source)
    yolo_results_lit = YoloPredictor.detect(source)

    camLightOff()
    detectron_results_dark = DetectronPredictor.detect(source)
    yolo_results_dark = YoloPredictor.detect(source)

    return [{'Light': (detectron_results_lit, yolo_results_lit)} ,
            {'Dark':(detectron_results_dark, yolo_results_dark)}]
    
def writePayloadData(results, ser):
    """ Takes CV inference results from the model and writes the data to the S/C FC
        via UART serial protocol
    Arguments
    ----------
    results : string
        A properly formatted dataframe in the format of a string
        TODO: results is probably a dataframe and must be converted into a string and
        then converted into bytes for the input buffer
    ser : Serial
        A Serializable object that performs UART comms to the main flight S/C computer
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
    #counter = 0
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
        #counter += 1
        if(bytesInBuffer is None):
            packetStatus = True
            break
    ser.flush()
    return packetStatus

def readStateVariable(self, file, state_var):
    """ Reads a JSON file that holds the state variables and returns a string set based on the 
        requested input message

        file : string
            file is a link to the filepath holding the json state variables
        state_var : string 
            state_var is the desired state variable that we want to read a value
            for
        out_val : tuple
            out_val is a string that holds the value read
    """
    temp = {} # temporary Python dictionary to hold state variables
    try:
        #data = json.load(file)
        with open(file, 'r') as json_out:
            temp = json.loads(json_out.read()) # gets a string, converts to dict
            print(temp)
    except Exception:
        print("Error reading the state variable file")
        return ("NO_VALUE", False) 

    for state in temp: # USED FOR DEBUGGING ONLY
        print(state)    # prints all of the state variables

    if state_var in temp:
        print("Here is the value for ", state_var)
        print(temp[state_var])
        return (temp[state_var], True)
    else:
        print("Could not locate the State Variable!")
        return("EMPTY STATE", False)

def writeStateVariable(self, file, state_var, new_val):
    """ Writes to a JSON file that holds the

        file : string
            file is a link to the filepath holding the json state variables
        state_var : string 
            state_var is the desired state variable that we want to read a value
            for
        Return:
            True if the new state was successfully found and written to the 
            state variables file. False if unsuccessful.
    """
    temp = {} # temporary dctionary python object to hold json state information
    try:
        with open("test_json.json", 'r') as json_out:
            new_msg = json.loads(json_out.read())
            #print(type(new_msg["name"]))
            new_msg.update(state_var=new_val)
            temp = new_msg
        with open("test_json.json", 'w') as json_out:
            json_out.write(json.dumps(temp))
    except Exception:
        print("Issue with writing the State Variables!")
        return False

    with open(file, 'r') as f: #Debugging statement to read what was written
        print(f.read())
    return True

def setup(self):
    """ Performs initial bootup sequence once and deploys the payload mechanism
        Checks status of components by writing to a json file
    Parameters
    ----------
    val: 
    """
    ser = serialSetup() # Setup UART protocol with main flight computer

    # Write to the STATE_VARIABLE json:
    #   -Increase the boot counter

    boot_counter_str = readStateVariable(state_variables_path, "BOOT_COUNTER")

    # Attempt to read the boot counter and cast to an integer, assumes that the boot counter 
    # is called correctly
    try:
        current_num_boots = int(boot_counter_str[0])
    except TypeError as e:
        print("Issue with reading for the state variable: BOOT_COUNTER!")
        print(e)
        current_num_boots = 0

    # Increment the boot counter by one
    current_num_boots += 1
    # Write the new number of boots to the state variables
    writeStateVariable(state_variables_path, "BOOT_COUNTER", current_num_boots)


    read_out = readStateVariable(state_variables_path, "DEPLOYED") # Check for the state variable for DEPLOYED

    try:
        deployed = read_out[0] # Get the state variable for Deployed
    except TypeError as e:
        print("ISSUE WITH READING THE STATE VARIABLE: DEPLOYED")
        print(e)
    
    # If deployed is FALSE, create a Burnwire() object and invoke the burn function
    # Runs .burn() for both pins at 5000 Hz for 1 second

    # TODO: Check if the Burnwire object exists. Only want to create a single Burnwire object once!
    if deployed is False and deployed is not None:
        #burnwire_1 = Burnwire(1, 5000, 0)
        burnwire = Burnwire(2, 5000, 0)
        #burnwire_1.getBurnwireStatus()
        burnwire.getBurnwireStatus()
        burnwire.burn(1, 100, 5000, 1) # Start pin 1 burn routine
        burnwire.burn(2, 100, 5000, 1) # Start pin 2 burn routine
        burnwire.destroy()
        writeStateVariable(state_variables_path, "BURNWIRE_FIRED", True) # Sets State Variable for burnwire fire event to TRUE
    elif deployed is True:
        # Move on to Define and Initialize Systems
        pass

    # Loop to check the Photodiode. Should this run as a parallel process?

    # While the Photodiode is "dark" and DEPLOYED is FALSE
        # Increment NUMBER_DARK_READINGS++

        # If NUMBER_DARK_READINGS > 10

            # Run the Burnwire again

            # Reset NUMMBER_DARK_READINGS = 0

            # Sleep 5 minutes
        
    # Set DEPLOYED to TRUE
                
    if not verifySystem():
        reboot()

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
