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

Req: 
Python 3.8.10
pySerial
"""
# =====================================
# ==         CONFIGURATION           ==
# =====================================

import os
from os import *
from time import time
import sys
# from loguru import RotationFunction, logger
# from pathlib import Path, WindowsPath5

import numpy as np
import math
#import loguru
import RPi.GPIO as GPIO
import serial
import threading
import json
import cv2 as cv
from PIL import Image

from serial.serialutil import SerialException

from Burnwire import Burnwire
from DriverLED import DriverLED
from Camera import Camera

sys.path.insert(0, os.getcwd() + '/FSW')
sys.path.insert(0, os.getcwd() + '/Detectron2')
sys.path.insert(0, os.getcwd() + 'YOLOv5')

import Timer
from Camera import *
from DetectronPredictor import *
#from YoloPredictor import run



# =====================================
# ==         DEPLOYMENT VARS         ==
# =====================================
PHOTODIODE_PIN = 9 # Photodiode GPIO20
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
SYS_TIMEOUT = 5 # seconds
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

EXPOSURE_TIME = 8000 # ms
DELAY = 3000 # ms
GAIN = 10
TIMEOUT = 10000
WIDTH = 800 # px
HEIGHT = 600 # px

PiCamera = None

# =====================================
# ==        MISC GLOBAL VARS         ==
# =====================================
SCIENCE = False
MAX_TEMP = 85 # measured in degrees C (see datasheet -20C - +85C)
INTERVAL_LENGTH = 10000 # 10 minutes - time between image captures
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
    """ Helper function
    """
    if n < minn:
        return minn
    elif n > maxn:
        return maxn
    else:
        return n

# TODO Write function that checks the photodiode, return True if above brightness
# threshold, return False otherwise

def checkPhotodiode(GPIO_PIN):
    """
    Parameters
    ----------
    GPIO_PIN : const int 
        GPIO pin attached to photodiode
    photodiode_reading : float 
        voltage reading from GPIO photodiode pin

    TODO: Determine the maximum photodiode value and change 1024 to constant value
    """
    photodiode_reading = GPIO.input(GPIO_PIN)

    if photodiode_reading is None:
        try:
            raise Exception('Photodiode I/O')
        except Exception as inst: # Get the exception instance
            print(inst.args, "\n \n")
            print("\nAttempted to access photodiode pin but no value?")
            print("\n Check if photodiode connected to correct pin?")
        return False

    if photodiode_reading is not type(float):
        if photodiode_reading > 1024 or photodiode_reading < 0:
            photodiode_reading = clamp(photodiode_reading, 0, 1024)

        if photodiode_reading > PHOTODIODE_THRESH:
            return True
    return False


# def checkDeployed():
#     """ Determines if the payload mechanism has been triggered and deployed by
#         comparing ambient light to some static threshold value
        
#     Parameters
#     ----------
#     val: float
#         Contains the digital photodiode readings
    
#     Returns
#     ---------
#     boolean output based on if deploy criterion met
#     """
#     if checkPhotodiode(PHOTODIODE_PIN):
#         return False
#     else:
#         return True

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

def checkVoltage():
    result = system('vcgencmd measure_volts core')
    print("Current voltage running in: core", result)
    return result

def reboot():
    """ Shell command to restart the RPi
    """
    system('sudo restart')

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

    # detectron_results_lit = DetectronPredictor.detect(source)
    # yolo_results_lit = YoloPredictor.detect(source)


    # detectron_results_dark = DetectronPredictor.detect(source)
    # yolo_results_dark = YoloPredictor.detect(source)

    # return [{'Light': (detectron_results_lit, yolo_results_lit)} ,
    #         {'Dark':(detectron_results_dark, yolo_results_dark)}]
    
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
        # Trigger Burnwire
        burnwire = Burnwire(2, 5000, 0)

        burnwire.getBurnwireStatus()
        burnwire.burn(1, 100, 5000, 1) # Start pin 1 burn routine
        burnwire.burn(2, 100, 5000, 1) # Start pin 2 burn routine
        burnwire.destroy()

        writeStateVariable(state_variables_path, "BURNWIRE_FIRED", True) # Sets State Variable for burnwire fire event to TRUE
    elif deployed is True:
        # Move on to Define and Initialize Systems
        # TODO DEFINE AND INITIALIZE SYSTEMS
        pass
    
    # INITIALIZE PAYLOAD CAMERA 
    PiCamera = Camera(exp=EXPOSURE_TIME, timeout=TIMEOUT, gain=GAIN, 
                        delay=DELAY, height=HEIGHT, width=WIDTH)

    # # Loop to check the Photodiode. Should this run as a parallel process?
    # photodiode_state = checkPhotodiode(PHOTODIODE_PIN)

    # # Get the number of dark readings so far
    # number_dark_readings = readStateVariable(state_variables_path, 
    #                                                 "NUMBER_DARK_READINGS")
    # # While the Photodiode is "dark" and DEPLOYED is FALSE
    # while photodiode_state is not True and deployed is False:
    #     # Increment NUMBER_DARK_READINGS++
    #     number_dark_readings += 1
    #     writeStateVariable(state_variables_path, "NUMBER_DARK_READINGS", number_dark_readings)
    #     # If NUMBER_DARK_READINGS > 10
    #     if (number_dark_readings % 10) == 0 :
    #         # Run the Burnwire again
    #         # Trigger Burnwire
    #         burnwire = Burnwire(2, 5000, 0)

    #         burnwire.getBurnwireStatus()
    #         burnwire.burn(1, 100, 5000, 1) # Start pin 1 burn routine
    #         burnwire.burn(2, 100, 5000, 1) # Start pin 2 burn routine
    #         burnwire.destroy()
    #         # Reset NUMMBER_DARK_READINGS = 0
    #         # number_dark_readings = 0
    #         # Sleep 5 minutes
    #         time.sleep(50000)
        
    # Set DEPLOYED to TRUE
    writeStateVariable(state_variables_path, "DEPLOYED", True)

    if not verifySystem():
        reboot()
    pass
  

if __name__ == "__main__":
    
    # Run the system setup function after 5 seconds
    timer = Timer(5.0, setup)

    # Initialize the timer
    timer.start()
    # Read the temperature?
    cpu_check = checkTempCPU()
    # Read the battery voltage?
    checkVoltage()
    # Check serial communication is good

    system_check = (cpu_check or serial.is_open)
    # Send health data packet to primary flight computer for check
    writeStateVariable(state_variables_path, "HARDWARE_ERROR", system_check)

    image_path = picam.getCapturePath()

    # Main Science FSW 
    while True:
        # if the elapsed time on the timer thread is a multiple of the timer interval 
        # threshold, perform data aquisition
        elapsed_time = math.ceil(timer.elapsed_time() % INTERVAL_LENGTH)

        if (elapsed_time == 0):
            # Take a picture
            img_capture = picam.takePicture("test{num}_gain_{gain}.jpg".format(num = picam.getNumPicsTake(), 
                                                                gain = picam.getGainVal()))
            # Push results of data aquisition to a local folder and return the path
            if img_capture[0]:
                image_path = img_capture[1]
                # Debugging: show the image
                cv.imshow(image_path)
                cv.waitKey(0)
                cv.destroyAllWindows()

                DetectronPredictor.detect(image_path)
            elif img_capture[1]:
                print(" IMAGE NOT CAPTURED! Trying again...")
                continue
            # Perform model inference on the returned path from the model

            
            # results = performInference(imgPath)


            # writePayloadData(results)


            #TODO: must be able to read from Rx and implement parity bit outcome
            pass
    ser.__exit__()
