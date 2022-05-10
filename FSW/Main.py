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
import os
import time
from time import sleep
import sys
import numpy as np
import RPi.GPIO as GPIO
import serial
# import threading
import json
import cv2 as cv
import subprocess
from PIL import Image

# Import JSC Payload and packages
from JSC_FLIGHT_HDD_EXP import HDD_ccw_drive, HDD_cw_drive, HDD_stop
from serial.serialutil import SerialException

from Burnwire import Burnwire
# from Camera import Camera

# Model Paths
sys.path.insert(0, os.getcwd() + '/FSW')
sys.path.insert(0, os.getcwd() + '/Detectron2')
sys.path.insert(0, os.getcwd() + 'YOLOv5')
from DetectronPredict import Inference_Mask
#from YoloPredictor import run

from FSWTimer import *

# =====================================
# ==         DEPLOYMENT VARS         ==
# =====================================
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
# Ramp parameters
HDD_SLEEP_TIME = 2.0
HDD_DELTA = 50

# =====================================
# ==        MISC GLOBAL VARS         ==
# =====================================
SCIENCE = False
MAX_TEMP = 85 # measured in degrees C (see datasheet -20C - +85C)
INTERVAL_LENGTH = 10000 # 10 minutes - time between image captures

parameterDB_path_name = '\home\pi\Desktop\ParameterDB'
STATE_VAR_NAME = "STATE_VARIABLES.json"
STATE_VAR_PATH = os.getcwd() + '/' + STATE_VAR_NAME
init_file = ""
deployed = False
burnwireFired = False
global_timer = None

def clamp(n, minn, maxn):
    """ Helper function
    """
    if n < minn:
        return minn
    elif n > maxn:
        return maxn
    else:
        return n

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

# def verifySystem():
#     """ Performs system diagnotic checks on power, temperature, lighting, and comms
#     Arguments
#     ----------
#     serial: Serial
#         Serial object returns the status of the serial connection to the S/C FC, can ping
#         for timeout and data packet state
#     Returns
#     ---------
#     False if any system related check function either also returns false or times out
#     True if all systems check out to true
#     """
#     # TODO: Perform RAM check, VRAM check, Power check, Lighting check, Camera check
#     if not serial.is_open:
#         return False
#     if not checkTempCPU():
#         return False
#     else:
#         return True

def getCPUTemp():
    """ Checks the SoC temperature in the core processor
    Parameters
    ----------
    output: string
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

def getInference(source):
    """ Calls CV models one after another, ensuring that light and dark lighting
        conditons are also created for each CV model
    TODO: Need to determine fault handling for if a model fails to perform successful inference
          i.e. memory leaks or failed inference.
    Arguments
    ---------
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
    buffer_as_bytes: bytes
        Bytes still in the input buffer
    Exceptions
    ---------
    SerialTimeoutException: Exception raised by Serial object if the connection
    times out, if so, restart the system serial connection.
    Returns
    ---------
    True if the data packet was sent without any issues
    False if otherwise
    """
    buffer_as_bytes = str.encode("Test Data", 'utf-8')
    # buffer_as_bytes = str.encode(results)
    print(type(buffer_as_bytes)) # check that it is a byte representation
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
            ser.write(buffer_as_bytes)
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
    parsed = {} # temporary Python dictionary to hold state variables
    try:
        with open(file, 'r') as json_out:
            parsed = json.loads(json_out.read()) # gets a string, converts to dict
    except Exception as e:
        print("Error reading the state variable file")
        print("Exception: ", e)
        return ("NO_VALUE", False) 
    # for state in parsed: # USED FOR DEBUGGING ONLY
    #     print(state)    # prints all of the state variables
    if state_var in parsed:
        return (parsed[state_var], True)
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
    parsed = {} # Temporary dctionary python object to hold json state information
    try:
        with open(file, 'r') as json_out:
            new_msg = json.loads(json_out.read())
            new_msg[state_var] = new_val
            parsed = new_msg
        with open(file, 'w') as json_out:
            json_out.write(json.dumps(parsed))
    except Exception:
        print("Issue with writing the State Variables!")
        return False

    # with open(file, 'r') as f: # Debugging statement to read what was written
    #     print(f.read())
    return True

def getSpacecraftState(file):
    try:
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
    print("Initial Gyro X:%.2f, Y: %.2f, Z: %.2f rads/s" % (w_x0, w_y0, w_z0))
    print("Final Gyro X:%.2f, Y: %.2f, Z: %.2f rads/s" % (w_xf, w_yf, w_zf))
    return [w_x0, w_y0, w_z0, w_xf, w_yf, w_zf]

def setup():
    """ Performs initial bootup sequence once and deploys the payload mechanism
        Checks status of components by writing to a json file
    """
    # Initialize the payload clock
    global_timer = FSWTimer()
    global_timer.start()
    
    # INITIALIZE F/C GPIO
    initializeComputer()

    # Attempt to read and increment the boot counter
    boot_counter_str = readStateVariable(STATE_VAR_PATH, "BOOT_COUNTER")
    try:
        current_num_boots = int(boot_counter_str[0])
    except Exception as e:
        print("Issue with reading for the state variable: BOOT_COUNTER!")
        print(e)
        print(boot_counter_str)
        current_num_boots = 0
    current_num_boots += 1
    writeStateVariable(STATE_VAR_PATH, "BOOT_COUNTER", current_num_boots)

    # Check if we already deployed
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
    
    # Debugging statement
    getSpacecraftState(STATE_VAR_PATH)
    
    print("+++++++++ F/C HEALTH CHECK +++++++++++++++")
    sleep(2)
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
        
    # Check serial UART connection is good
    ser_is_open = writeData("Testing Setup")
    if not ser_is_open:
        print("Issue with connecting to S/C Serial port!")
        system_error = True
    
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

from tabulate import tabulate
TIME_PER_HIO = 5*60 # minutes
MAX_IMGS = 2
model_filename = os.getcwd()+'/handrail_output.pth'    #located in output.zip folder

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
    HIO_timer = FSWTimer()
    HIO_timer.start()
    
    number_imgs_captured = 0

    while number_imgs_captured <= MAX_IMGS-1:
        # if the elapsed time on the timer thread is a multiple of the timer interval 
        # threshold, perform data aquisition
        if (HIO_timer.elapsed_time() % TIME_PER_HIO >= 0):
            # Take a picture
            # img_capture = picam.takePicture("test{num}_gain_{gain}.jpg".format(num = picam.getNumPicsTake(), 
            #                                                     gain = picam.getGainVal()))
            
            # Test dummy data
            img_capture = os.getcwd() + '/sample_img_2.jpg'
            img = cv.imread(img_capture)
            
            # Push results of data aquisition to a local folder and return the path
            if img_capture is not None:
                # Debugging: show the image
                cv.imshow("debug img", img)
                cv.waitKey(0)
                cv.destroyAllWindows()

                # Perform model inference on the returned path from the model
                # DetectronPredictor.detect(image_path)
                
                # im = cv2.imread(os.getcwd()+"/sample_img.jpg")     #located in same directory as inference_mask.py
                inference_threshold = 0.02
                det_dict = Inference_Mask(img, inference_threshold)

                ### TABULATED DETECTION RESULTS ###
                spc1 = "                 "
                spc2 = "              "

                # Includes IOU and TP?
                headers = ["Detection","Bbox"+spc1+spc1+spc1+"Conf"+ spc2+"IOU"+spc2+"TP?"] 
                table = tabulate(det_dict.items(),headers = headers)
                print(table)
                
                results_str = str(det_dict)
                writeData(results_str)
                
                
                
            elif img_capture is None:
                print(" IMAGE NOT CAPTURED! Trying again...")
                continue
            
            # writePayloadData(results)
            number_imgs_captured += 1
        
  

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
    
