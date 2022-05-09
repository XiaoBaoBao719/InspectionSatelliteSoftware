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


TODO: CREATE A PINOUT TXT OR JSON THAT SETS ALL OF THE PIN LOCATIONS 

"""
# Use the correct version of Python...
#!/usr/bin/Python3

# =====================================
# ==         CONFIGURATION           ==
# =====================================

# from distutils.dep_util import newer_pairwise
import os
import time
from time import sleep
import sys
# from tkinter import NONE
# from loguru import RotationFunction, logger
# from pathlib import Path, WindowsPath5

import numpy as np
# import math
#import loguru
import RPi.GPIO as GPIO
import serial
# import threading
import json
import cv2 as cv
import subprocess
from PIL import Image

# Unique packages for the HDD Payload
# import board

# os.system ("sudo pigpiod") # start pigpio daemon before doing anything
# import pigpio

# import adafruit_ina260 # Measures the voltmeter
# import adafruit_icm20x # Measures the IMU
from JSC_FLIGHT_HDD_EXP import HDD_ccw_drive, HDD_cw_drive, HDD_stop

from serial.serialutil import SerialException

from Burnwire import Burnwire
# from DriverLED import DriverLED
# from Camera import Camera

sys.path.insert(0, os.getcwd() + '/FSW')
sys.path.insert(0, os.getcwd() + '/Detectron2')
sys.path.insert(0, os.getcwd() + 'YOLOv5')

from FSWTimer import *
# from Camera import *
# from DetectronPredict import Inference_Mask

#from YoloPredictor import run


# =====================================
# ==         DEPLOYMENT VARS         ==
# =====================================
# PHOTODIODE_PIN = 9 # Photodiode GPIO20
#INPUT_PIN = 10
#BURN_PIN_1 = 11
#BURN_PIN_2 = 12
#LED_PIN = 14
#BURNWIRE_PINS = (BURN_PIN_1, BURN_PIN_2)
# BURNWIRE_WAIT_TIME = 20 # mins
# PHOTODIODE_THRESH = 1 # We need to test photodiode to find a good value for this
NUM_BURNWIRES = 2

# =====================================
# ==       COMMS GLOBAL VARS         ==
# =====================================
BAUDRATE = 115200
SYS_TIMEOUT = 5 # seconds
mini_UART = '/dev/ttyS0'
PL011 = '/dev/ttyAMA0' 
SERIAL_PORT = PL011


# =====================================
# ==           CV VARS               ==
# =====================================
# FILE = Path(__file__).resolve()
# ROOT = FILE.parents[0]  # YOLOv5 root directory

# Detectron2_WEIGHTS = ROOT / 'detmodel.pt'
# Detectron2_SOURCE = ROOT / 'data/images'
# YOLO_WEIGHTS = ROOT / 'yolov5s.pt'
# YOLO_SOURCE = ROOT / 'data/images'

EXPOSURE_TIME = 8000 # ms
DELAY = 3000 # ms
GAIN = 10
TIMEOUT = 10000
WIDTH = 800 # px
HEIGHT = 600 # px

piCamera = None
burnwire = None

# =====================================
# ==        HDD GLOBAL VARS          ==
# =====================================
# i2c = board.I2C()
# imu = adafruit_icm20x.ICM20948(i2c)
# pi = pigpio.pi(); 

# ESC_PINOUT = 18

# pi.set_servo_pulsewidth(ESC_PINOUT, 0)

# ESC input range
# NEUTRAL_ESC_IN = 1488
# MAX_CCW_ESC_IN = 1132
# MAX_CW_ESC_IN = 1832


# Ramp parameters
HDD_SLEEP_TIME = 2.0
HDD_DELTA = 50

# =====================================
# ==        MISC GLOBAL VARS         ==
# =====================================
SCIENCE = False
MAX_TEMP = 85 # measured in degrees C (see datasheet -20C - +85C)
INTERVAL_LENGTH = 10000 # 10 minutes - time between image captures

current_time = 0
parameterDB_path_name = '\home\pi\Desktop\ParameterDB'
STATE_VAR_NAME = "STATE_VARIABLES.json"
STATE_VAR_PATH = os.getcwd() + '/' + STATE_VAR_NAME
init_file = ""
deployed = False
burnwireFired = False
global_timer = None


# def startTimer(timer):
#    """ TODO: initializes the global timer var as a seperate thread process
#    """
#     timer.start()

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

# def checkPhotodiode(GPIO_PIN):
#     """
#     Parameters
#     ----------
#     GPIO_PIN : const int 
#         GPIO pin attached to photodiode
#     photodiode_reading : float 
#         voltage reading from GPIO photodiode pin

#     TODO: Determine the maximum photodiode value and change 1024 to constant value
#     """
#     photodiode_reading = GPIO.input(GPIO_PIN)

#     if photodiode_reading is None:
#         try:
#             raise Exception('Photodiode I/O')
#         except Exception as inst: # Get the exception instance
#             print(inst.args, "\n \n")
#             print("\nAttempted to access photodiode pin but no value?")
#             print("\n Check if photodiode connected to correct pin?")
#         return False

#     if photodiode_reading is not type(float):
#         if photodiode_reading > 1024 or photodiode_reading < 0:
#             photodiode_reading = clamp(photodiode_reading, 0, 1024)

#         if photodiode_reading > PHOTODIODE_THRESH:
#             return True
#     return False


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
    print("\n ++++++++ STARTING FLIGHT COMPUTER +++++++++++")
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

# This whole function was a terrible idea. You can do better Xiao!
# def serialSetup():
#     """ 
#     Parameters
#     ----------
#     ser: Serial
#         Serial object handles UART comms protocol with S/C
#     
#     Return
#     ---------
#         Serial object if successful connection and setup established, 
#         otherwise, return None
#     """
#     
#     if ser == None:
#         # Initialize Serial Port
#         print("\nAttemtping to open the serial port...")
#         ser = ""
#         try:
#             ser = serial.Serial(port=SERIAL_PORT, baudrate=BAUDRATE,
#                                 parity=serial.PARITY_ODD,timeout=SYS_TIMEOUT,
#                                 stopbits=serial.STOPBITS_ONE,bytesize=serial.EIGHTBITS)
#             print(ser.name)
#             ser.write(b'CV_Payload_Active')
#             ser.open()
#             print("Serial connection successful! ", ser.name)
#         except SerialException as e:
#             print("Issue with setting up UART connection!")
#             print(e)
#             ser = None
#     #counter = 0
#     if ser.is_open:
#         print("Serial port already open.")
#         print(ser.name)
#         return ser
#     
#     if ser is not None:
#         return ser
# 
#     return None

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
    if not checkTempCPU():
        return False
    else:
        return True

def getCPUTemp():
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
    output = subprocess.check_output(["vcgencmd", "measure_temp", "core"])
    # print(type(output))
    temperature = float(output[5:9])
    print('Current CPU Temp is :', temperature, " *C")
    return temperature
    

def getVoltage():
    output = subprocess.check_output(['vcgencmd', 'measure_volts', 'core'])
    voltage = float(output[5:11])
    print("Current voltage running in: core", voltage, 'V')
    return voltage

def reboot():
    """ Shell command to restart the RPi
    """
    print("Rebooting in 10 seconds")
    reboot_time = FSWTimer()
    reboot_time.start()
    while reboot_time.elapsed_time() < 10:
        time_left = 10 - reboot_time.elapsed_time()
        print(f"Rebooting in: {time_left:0.2f}")
    sleep(0.1)
    # system('sudo restart')
    sys.exit(0)

def getInference(model, source):
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
    
def writeData(results):
    """ Takes CV inference results from the model and writes the data to the S/C FC
        via UART serial protocol
    Arguments
    ----------
    results : string
        A properly formatted dataframe in the format of a string
        TODO: results is probably a dataframe and must be converted into a string and
        then converted into bytes for the input buffer
    Parameters
    ----------
    bytesInBuffer: int
        Number of bytes still in the input buffer
    Exceptions
    ---------
    SerialTimeoutException: Exception raised by Serial object if the connection
    times out, if so, restart the system serial connection.
    Returns
    ---------
    True if the data packet was sent without any issues
    False if otherwise
    """
    buffer = str(results)
    
    with serial.Serial() as ser:
        ser.port=SERIAL_PORT
        ser.baudrate=BAUDRATE
        ser.parity=serial.PARITY_NONE
        ser.timeout=SYS_TIMEOUT
        ser.stopbits=serial.STOPBITS_ONE
        ser.bytesize=serial.EIGHTBITS
        ser.open()

        try:
            print("Writing results")
            ser.write(buffer)
            print("Packet sent")
            time.sleep(1) # Wait one second for packet to send
            return True
            
        except (serial.SerialTimeoutException):
            # Reset the buffer and reset the serial conection
            print("Serial Timed out! Re-attempting connection...")
            ser.reset_input_buffer()
            ser.reset_output_buffer()
    
        # bytesInBuffer = ser.in_waiting()
        # print(bytesInBuffer)
        # print(ser.out_waiting())
        
        #counter += 1
        # if(bytesInBuffer is None):
        #     packetStatus = True
        #     return packetStatus
    ser.close()
    ser.flush()
    return False

def readStateVariable(file, state_var):
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
            # print(temp)
    except Exception as e:
        print("Error reading the state variable file")
        print("Exception: ", e)
        return ("NO_VALUE", False) 

    # for state in temp: # USED FOR DEBUGGING ONLY
    #     print(state)    # prints all of the state variables

    if state_var in temp:
    #     print("Here is the value for ", state_var)
    #     print(temp[state_var])
        return (temp[state_var], True)
    else:
        print("Could not locate the State Variable!")
        print("State variable: ", state_var)
        return("EMPTY STATE", False)

def writeStateVariable(file, state_var, new_val):
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
    temp = {} # Temporary dctionary python object to hold json state information
    try:
        with open(file, 'r') as json_out:
            new_msg = json.loads(json_out.read())
            #print(type(new_msg["name"]))
            # print("Update state: ", state_var)
            # print("New value will be: ", new_val)
            # new_msg.update(state_var=new_val)
            new_msg[state_var] = new_val
            # print("\n New value for states: ")
            # print(new_msg)
            temp = new_msg
        with open(file, 'w') as json_out:
            json_out.write(json.dumps(temp))
    except Exception:
        print("Issue with writing the State Variables!")
        return False

    # with open(file, 'r') as f: # Debugging statement to read what was written
    #     print(f.read())
    return True

def getSpacecraftState(file):
    temp = {} # temporary Python dictionary to hold state variables
    try:
        #data = json.load(file)
        with open(file, 'r') as json_out:
            parsed = json.loads(json_out.read()) # gets a string, converts to dict
            print(json.dumps(parsed, indent=4, sort_keys=False))
    except Exception as e:
        print("Error reading the state variable file")
        print("Exception: ", e)
        return ("NO_VALUE", False)
    
"""
def setupBurnwire(wire, deployed):
    # TODO: Check if the Burnwire object exists. Only want to create a single Burnwire object once!
    if wire is None:
        try:
            wire = Burnwire(2, 5000, 0)
        except RuntimeError as r:
            print("Issue with creating Burnwire object!")
            print("\n", r)

        if deployed is False and deployed is not None:
            # Trigger Burnwire
            wire.getBurnwireStatus()
            wire.burn(1, 100, 5000, 1) # Start pin 1 burn routine
            wire.burn(2, 100, 5000, 1) # Start pin 2 burn routine
            wire.destroy()

            writeStateVariable(STATE_VAR_PATH, "BURNWIRE_FIRED", True) # Sets State Variable for burnwire fire event to TRUE
        elif deployed is True:
            # Move on to Define and Initialize Systems
            # TODO DEFINE AND INITIALIZE SYSTEMS
            pass
    elif wire is not None:
        print("Burnwire object already created!")
"""

def doHDD():
    # [w_x0, w_y0, w_z0, w_xf, w_yf, w_zf] = HDD_ccw_drive(sleep_time, delta)   uncomment when testing!
    # HDD_stop()
    [w_x0, w_y0, w_z0, w_xf, w_yf, w_zf] = [-1.0, -2.0, -3.0, -1.0, -2.0, -3.0]
    #dummy_hdd_list = np.random.rand(6,0).tolist()
    #print(dummy_hdd_list)
    #[w_x0, w_y0, w_z0, w_xf, w_yf, w_zf] = dummy_hdd_list
    print("Initial Gyro X:%.2f, Y: %.2f, Z: %.2f rads/s" % (w_x0, w_y0, w_z0))
    print("Final Gyro X:%.2f, Y: %.2f, Z: %.2f rads/s" % (w_xf, w_yf, w_zf))
    return [w_x0, w_y0, w_z0, w_xf, w_yf, w_zf]

def setup():
    """ Performs initial bootup sequence once and deploys the payload mechanism
        Checks status of components by writing to a json file
    """
    # INITIALIZE F/C GPIO
    initializeComputer()

    # Setup UART protocol with main flight computer
    # ser = serialSetup()
    # This is actually a terrible idea. We should only open the serial connection when transmitting data.

    # Increase the boot counter
    boot_counter_str = readStateVariable(STATE_VAR_PATH, "BOOT_COUNTER")
    # Attempt to read the boot counter
    try:
        current_num_boots = int(boot_counter_str[0])
    except Exception as e:
        print("Issue with reading for the state variable: BOOT_COUNTER!")
        print(e)
        print(boot_counter_str)
        current_num_boots = 0
    # Increment the boot counter by one
    current_num_boots += 1
    # Write the new boot counter
    writeStateVariable(STATE_VAR_PATH, "BOOT_COUNTER", current_num_boots)

    # Check for the state variable for DEPLOYED
    read_out = readStateVariable(STATE_VAR_PATH, "DEPLOYED") 
    try:
        deployed = read_out[0] # Get the state variable for Deployed
    except TypeError as e:
        print("ISSUE WITH READING THE STATE VARIABLE: DEPLOYED")
        print(e)

    # INITIALIZE PAYLOAD CAMERA 
    """
    if PiCamera is None:
        try:
            PiCamera = Camera(exp=EXPOSURE_TIME, timeout=TIMEOUT, gain=GAIN, 
                                delay=DELAY, height=HEIGHT, width=WIDTH)
        except RuntimeError:
            print("Could not create PiCamera object!")
    """
    
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

    # Debugging statement
    getSpacecraftState(STATE_VAR_PATH)
    
    # CHECK SYSTEM HEALTH

    # Initialize the payload clock
    global_timer = FSWTimer()
    global_timer.start()
    
    # Assume no system errors yet
    system_error = False
    
    # Check CPU is not too hot
    cpu_temp = getCPUTemp()
    if cpu_temp >= MAX_TEMP:
        print(f"CPU temp is {cpu_temp:0.2f}")
        print('CPU is too hot!')
        system_error = True
        
    # Check voltage is not too low
    voltage = getVoltage()
    # voltage = 0.01  for testing voltage
    
    if voltage <= 0.1:
        print(f"Voltage is {voltage:0.2f}") 
        print("Voltage too low!")
        system_error = True
    # Check system health is good
    
    # Send health data packet to primary flight computer for check
    writeStateVariable(STATE_VAR_PATH, "HARDWARE_ERROR", system_error)

    if system_error:
        print("Anomally detected! Rebooting...")
        sleep(5)
        reboot()
    
    print("Health Check Complete. \n Standing by")
    sleep(5)
    
TOTAL_HDD_EXPERIMENTS = 2 # number experiments to perform
TIME_PER_HDD = 10 # 5 mins between each experiment

def HDD_Main():
    # Start HDD Experiment
    print("Initiating HDD experiment...")
    sleep(5)
    HDD_results = []
    num_experiments = 1
    HDD_timer = FSWTimer()
    HDD_timer.start()
    
    while num_experiments <= TOTAL_HDD_EXPERIMENTS:
        if (HDD_timer.elapsed_time() % TIME_PER_HDD >= 0):
            print("Running HDD experiment: ", num_experiments)
            print(f"Elapsed time: {HDD_timer.elapsed_time():0.4f} seconds")
            experiment_run = doHDD()
            HDD_results.append(experiment_run)
            
            print("Run successful!")
            # sleep(TIME_PER_HDD) # Wait for system to settle after 5 mins

        num_experiments += 1
    
    print(f"Elapsed HDD time: {HDD_timer.elapsed_time():0.4f} seconds")
    HDD_timer.stop()
    # Write HDD results to UCD Data buffer
    print("Writing HDD data to buffer...")
    writeData(HDD_results)
    pass

def HIO_Main():
    # If deployed is FALSE, create a Burnwire() object and invoke the burn function
    # Runs .burn() for both pins at 5000 Hz for 1 second
    read_out = readStateVariable(STATE_VAR_PATH, "DEPLOYED") # Check for the state variable for DEPLOYED

    try:
        deployed = read_out[0] # Get the state variable for Deployed
    except TypeError as e:
        print("ISSUE WITH READING THE STATE VARIABLE: DEPLOYED")
        print(e)
    
    print("Checking deployed state: ", deployed)
    
    if deployed == False and burnwire is None:
        # SETUP AND DEPLOY BURNWIRE 
        burnwire = Burnwire(NUM_BURNWIRES)
        burnwire.getBurnwireStatus()
        sleep(2)
        burn_channels = [1, 2]
        burn_result = burnwire.burn(burn_channels)
        if burn_result:
            print("Burn successful!")
            # Set DEPLOYED to TRUE
            writeStateVariable(STATE_VAR_PATH, "BURNWIRE_FIRED", True)
            writeStateVariable(STATE_VAR_PATH, "DEPLOYED", True)
            
        else:
            print("Burn attempt failed!")
    
    # image_path = picam.getCapturePath()
    
    getSpacecraftState(STATE_VAR_PATH)
    
    print("Initiating HIO Experiment...")
    sleep(5)
    """
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

                # DetectronPredictor.detect(image_path)
            elif img_capture[1]:
                print(" IMAGE NOT CAPTURED! Trying again...")
                continue
            # Perform model inference on the returned path from the model

            
            # results = performInference(imgPath)


            # writePayloadData(results)


            #TODO: must be able to read from Rx and implement parity bit outcome
            pass
        """
  

if (__name__ == "__main__"):

    # Run system setup
    setup()

    HDD_Main()
    sleep(10) # Wait 10 minutes for steady-state
    HIO_Main()
    
    # Clean up system
    #ser.close()
    os.system ("sudo killall pigpiod")
    print("Goodbye!")
    sleep(2)
    #ser.__exit__()
    
