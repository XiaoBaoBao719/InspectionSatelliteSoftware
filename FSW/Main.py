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
Python 3.7.3
pySerial
TODO: CREATE A PINOUT TXT OR JSON THAT SETS ALL OF THE PIN LOCATIONS 
"""
# Use the correct version of Python...
#!/usr/bin/env Python3
import sys
#testing importing all modules
#sys.path.insert(0, '/home/pi/.local/lib/python3.7/site-packages')
# =====================================
# ==         CONFIGURATION           ==
# =====================================
import os
import time
import numpy as np
import RPi.GPIO as GPIO
import serial
import json
import cv2 as cv
import subprocess
import csv
from PIL import Image

# TESTING: I2C EXTENDED BUS
#from adafruit_extended_bus import ExtendedI2C as I2C

from time import sleep
from tabulate import tabulate

# Import JSC Payload and packages
os.system ("sudo pigpiod")
from JSC_FLIGHT_HDD_EXP_V2 import HDD_ccw_drive, HDD_cw_drive, HDD_stop, HDD_print
from serial.serialutil import SerialException

from Burnwire import Burnwire

# Model Paths
sys.path.insert(0, os.getcwd())
sys.path.insert(0, os.getcwd() + '/Detectron2')
sys.path.insert(0, os.getcwd() + 'YOLOv5')
from DetectronPredict import Inference_Mask, displayResults, getBestResults
from YoloPredict import Inference_Yolo
from FSWTimer import *

from char2bits import char2bits
from bits2char import bits2char
from read_data_string import read_data_string
from write_data_string import write_data_string

#from FinalCam import run_camera

# =====================================
# ==         DEPLOYMENT VARS         ==
# =====================================
NUM_BURNWIRES = 2
ACCESSORY_PWR = 26

# =====================================
# ==       COMMS GLOBAL VARS         ==
# =====================================
BAUDRATE = 19200 #115200
SYS_TIMEOUT = 1 # seconds
mini_UART = '/dev/serial0'
PL011 = '/dev/ttyAMA0'
serial_zero = '/dev/serial0'
SERIAL_PORT = serial_zero

# =====================================
# ==           CV VARS               ==
# =====================================
EXPOSURE_TIME = 8000 # ms
DELAY = 3000 # ms
GAIN = 10
TIMEOUT = 10000
WIDTH = 800 # px
HEIGHT = 600 # px

#global PiCamera
PiCamera = None
burnwire = None

# =====================================
# ==        HDD GLOBAL VARS          ==
# =====================================
# Ramp parameters
HDD_SLEEP_TIME = 2.0
HDD_DELTA = 50
sleep_time = 2.0
delta = 50

# =====================================
# ==        MISC GLOBAL VARS         ==
# =====================================
SCIENCE = False
MAX_TEMP = 85 # measured in degrees C (see datasheet -20C - +85C)
INTERVAL_LENGTH = 10000 # 10 minutes - time between image captures
SLEEP_TIME = 10 # in seconds

parameterDB_path_name = '\home\pi\Desktop\ParameterDB'
STATE_VAR_NAME = "STATE_VARIABLES.json"
STATE_VAR_PATH = os.getcwd() + '/' + STATE_VAR_NAME
init_file = ""
global deployed
global hdd_done
global burnwire_fired
global global_timer 

HDD = 0 # Used for writing bitstream to JSC
HIO = 1 # Used for writing bitstream to JSC

LED = 21 # GPIO21, not used

def flashled(num):
    for i in range(num):
        GPIO.output(LED, GPIO.HIGH)
        sleep(1)
    GPIO.output(LED, GPIO.LOW)
    pass

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
    print("\n+++++++++++++++ STARTING FLIGHT COMPUTER +++++++++++++++")
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(LED,GPIO.OUT)
    GPIO.setup(ACCESSORY_PWR, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    

def getCPUTemp():
    """ Checks the SoC temperature in the core processor
    Parameters
    ----------
    output: string
        She``l command to pull measured processor temperature
    temperature: float
        CPU processing temperature in degrees Celsius by pulling a substring of output
    Returns
    ---------
    True of the processor is within a temperature range set by MAX_TEMP
    False if the processor temperature exceeds allowable (system will auto throttle)
    """
    #output = subprocess.check_output(["vcgencmd", "measure_temp", "core"])
    output = subprocess.check_output(["vcgencmd", "measure_temp"])
    temperature =  float(output[5:9])
    print('Current CPU Temp is :', temperature, " *C")
    return temperature
    
def getVoltage():
    output = subprocess.check_output(['vcgencmd', 'measure_volts'])
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
    os.system('sudo restart')
    sys.exit(0)
    
def writeData(results):
    """ Takes HIO or HDD results (character string) from the model and writes the data to the S/C FC
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
    ser = serial.Serial()
    #with serial.Serial() as ser: # Commented out for CM4, as it causes problems with the CM4
    ser.port=SERIAL_PORT
    ser.baudrate=BAUDRATE
    ser.parity=serial.PARITY_NONE
    ser.timeout=SYS_TIMEOUT
    ser.stopbits=serial.STOPBITS_ONE
    ser.bytesize=serial.EIGHTBITS
    
    result_mod = "(" + results + ")"
    try:
        print("Writing results...")
        ser.open()
        #time.sleep(1)
        #temp = results.encode('utf-8')
        #print(temp)
        
        num = ser.write(result_mod.encode('utf-8'))
        print(results.encode('utf-8'))
        print(num)
        time.sleep(0.1)
        print("Packet sent!")
        time.sleep(1) # Wait one second for packet to send
        ser.flush()
        ser.close()
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
    
    ser.flush()
    ser.close()
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
            print("\n--------------- S/C State ---------------")
            print(json.dumps(parsed, indent=4, sort_keys=False))
    except Exception as e:
        print("Error reading the state variable file")
        print("Exception: ", e)
        return ("NO_VALUE", False)



def doHDD(dir):
    if dir:
        result = HDD_ccw_drive(sleep_time=HDD_SLEEP_TIME, delta=HDD_DELTA)
    
    else:
        result = HDD_cw_drive(sleep_time=HDD_SLEEP_TIME, delta=HDD_DELTA)
    sleep(5)
    HDD_stop()   # uncomment when testing!
    #flashled(5)
    # [w_x0, w_y0, w_z0, w_x1, w_y1, w_z1, w_xf, w_yf, w_zf, HDD_current_f] = [-1.0, -2.0, -3.0, 1.0, 2.0, 3.0, -1.0, -2.0, -3.0, 0.05]
    # result = [-1.0, -2.0, -3.0, 1.0, 2.0, 3.0, -1.0, -2.0, -3.0, 0.05]
    HDD_print(result)
    return result

# def reference():
#     sleep(3)
#     print("\n CASE: \"It's not possible!\"")
#     sleep(2)
#     print("\n Cooper: \"No. It's necessary\"")
#     sleep(2)
#     print("\nSuspenseful Hans Zimmer plays...\n")
#     sleep(2)

def setup():
    """ Performs initial bootup sequence once and deploys the payload mechanism
        Checks status of components by writing to a json file
    """
    # Initialize globals
    deployed = False
    hdd_done = False
    burnwire_fired = False
    global_timer = None
    
    # INITIALIZE F/C GPIO
    initializeComputer()
    #flashled(5)
    
    # Initialize the payload clock
    global_timer = FSWTimer()
    global_timer.start()

    # INITIALIZE PAYLOAD CAMERA 
#     if PiCamera is None:
#         try:
#             PiCamera = Camera(exp=EXPOSURE_TIME, timeout=TIMEOUT, gain=GAIN, delay=DELAY, height=HEIGHT, width=WIDTH)
#         except RuntimeError:
#             print("Could not create PiCamera object!")
    
    # Debugging statement
    getSpacecraftState(STATE_VAR_PATH)
    
    print("\n+++++++++++++++ F/C HEALTH CHECK +++++++++++++++")
    sleep(0.2)
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
#     d = "COMMcheckWITHup"
#     ser_is_open = writeData(d)
#     if not ser_is_open:
#         print("Issue with connecting to S/C Serial port!")
#         system_error = True
    
    # Send health data packet to primary flight computer for check
    writeStateVariable(STATE_VAR_PATH, "HARDWARE_ERROR", system_error)

    if system_error:
        flashled(10)
        print("Anomally detected! Rebooting...")
        sleep(5)
        reboot()
    
    print("Health Check Complete. \nStanding by...")
    #flashled(20)
    sleep(0.2)
    
#TOTAL_HDD_EXPERIMENTS = 4  # number experiments to perform
TIME_PER_HDD = 1*60 # 5 mins between each experiment

def HDD_Main():
    # Start HDD Experiment
    print("Initiating HDD experiment...")
    #flashled(10)
    sleep(0.1)
    HDD_results = []
    HDD_timer = FSWTimer()
    #tmp_timer = FSWTimer()
    HDD_timer.start()
    #tmp_timer.start()
    
    curr_results = []
    hdd_bytes = ""

    #num_runs = 1  #TODO read this from state json
    read_out = readStateVariable(STATE_VAR_PATH, "NUMBER_HDD_RUNS")
    try:
        num_runs = read_out[0] # Get the state variable for NUMBER_HDD_RUNS
    except TypeError as e:
        print("ISSUE WITH READING THE STATE VARIABLE: NUMBER_HDD_RUNS")
        print(e)
    
    
    print("\n+++++++++++++++ STARTING HDD +++++++++++++++")
    sleep(0.1)
    #reference()
    if permission():
        #if (HDD_timer.elapsed_time() >= TIME_PER_HDD):
        print("\nRunning HDD experiment: ", num_runs)
        print(f"Elapsed time: {HDD_timer.elapsed_time():0.4f} seconds")
        
        curr_results = doHDD(num_runs%2 > 0.5)
        
        HDD_results.append(curr_results)
        
        print("Run successful!")
        print(f"Elapsed time: {HDD_timer.elapsed_time():0.4f} seconds")
        # sleep(TIME_PER_HDD) # Wait for system to settle after 5 mins
        wxa = curr_results[0]
        wxb = curr_results[3]
        wxc = curr_results[6]
        wya = curr_results[1]
        wyb = curr_results[4]
        wyc = curr_results[7]
        wza = curr_results[2]
        wzb = curr_results[5]
        wzc = curr_results[8]
        T_a = curr_results[9]
        T_b = curr_results[10]
        T_c = curr_results[11]
        cd  = curr_results[12]

        # Write HDD results to UCD Data buffer
        print("Writing HDD data to buffer...")
        hdd_bytes = write_data_string(TYPE=HDD, XN=num_runs, WXA=wxa, WXB=wxb, WXC=wxc, 
                                        WYA=wya, WYB=wyb, WYC=wyc, WZA=wza, WZB=wzb, WZC=wzc,
                                        TA=T_a,TB=T_b,TC=T_c,CD=cd, TEMP=getCPUTemp())
        hdd_data_string = bits2char(hdd_bytes)
        writeData(hdd_data_string)
        
        comm_path = os.getcwd() + '/comms_sent.csv'
        with open(comm_path, 'a', newline='') as comm_csv: # Write to csv file
            writer = csv.writer(comm_csv)
            writer.writerow([hdd_data_string])
        
        #tmp_timer = FSWTimer()
        #tmp_timer.start()
        # Update
        num_runs += 1
        writeStateVariable(STATE_VAR_PATH, "NUMBER_HDD_RUNS", num_runs)
        
    print(f"Total Elapsed HDD time: {HDD_timer.elapsed_time():0.4f} seconds")
    HDD_timer.stop()
    print("sleep HDD")
    time.sleep(1)
    #print("arise")
    pass


TIME_PER_HIO = 1*60 #5 minutes between experiments
#MAX_IMGS = 2
INFERENCE_THRESHOLD = 0.4
model_filename = os.getcwd()+'/handrail_output.pth'    #located in output.zip folder

def HIO_Setup():
    print("Setting up HIO Experiment...")
    sleep(0.2)
    getSpacecraftState(STATE_VAR_PATH)
    # If deployed is FALSE, create a Burnwire() object and invoke the burn function
    # Runs .burn() for both pins
    
    # if burnwire_fired == "false":
    # SETUP AND DEPLOY BURNWIRE
    burnwire = Burnwire(NUM_BURNWIRES)
    burnwire.getBurnwireStatus()
    sleep(1)
    burn_channels = [1, 2]

    # Write to state variable in case of burnwire forced reboot event
    writeStateVariable(STATE_VAR_PATH, "BURNWIRE_FIRED", True)
    
    # This is potentially dangerous unless we have another way of checking if we are deployed
    writeStateVariable(STATE_VAR_PATH, "DEPLOYED", True)
    
    deployed = burnwire.burn(burn_channels)
        
    # If we successfully deployed nominally
    if deployed:
        print("Burn successful!")
        writeStateVariable(STATE_VAR_PATH, "DEPLOYED", deployed)
    else:
        print("Burn attempt failed!")
        writeStateVariable(STATE_VAR_PATH, "DEPLOYED", deployed)
        
    getSpacecraftState(STATE_VAR_PATH)

def HIO_Main():

    print("Initiating HIO Experiment...")
    sleep(0.1)
    #HIO_timer = FSWTimer()
    #HIO_timer.start()
    
    num_imgs_str = readStateVariable(STATE_VAR_PATH, "NUMBER_IMAGES")
    try:
        num_imgs = int(num_imgs_str[0])
    except Exception as e:
        print("Issue with reading for the state variable: NUMBER_IMAGES!")
        print(e)
        print(num_imgs_str)
        num_imgs = 1

    print("\n+++++++++++++++ THE EYE OF HIO OPENS ITS BALEFUL GAZE +++++++++++++++")
        
    print("Using image - LIVE PHOTO")
    os.system("libcamera-jpeg -o sample.jpg --nopreview --rotation 180 -t10")
    img_capture = os.getcwd() + '/sample.jpg'
    
    img = cv.imread(img_capture)
    
    # Push results of data aquisition to a local folder and return the path
    if img_capture is not img:
        # Debugging: show the image
        # cv.imshow("debug img", img)
        # cv.waitKey(0)
        # cv.destroyAllWindows()

        # Perform model inference on the returned path from the model              
        mask_result = Inference_Mask(img, INFERENCE_THRESHOLD)
        yolo_result = Inference_Yolo(img)
        
        ### DEBUGGING TABULATED DETECTION RESULTS ###
        displayResults(mask_result)
        
        # Test Dummy Data
        #mask_result = {0: {'bbox': [371.8197, 312.28333, 1514.4973, 665.65515], 'conf': 0.99867}, 1: {'bbox': [998.0732, 295.11374, 1515.9216, 582.3098], 'conf': 0.07558}}
        
        # Test Debugging Dumb Mode
        # det_dict = {0: {'bbox': [371.8197, 312.28333, 1514.4973, 665.65515], 'conf': 0.99867}, 1: {'bbox': [998.0732, 295.11374, 1515.9216, 582.3098], 'conf': 0.07558}}

        mask_bb, mask_conf = getBestResults(mask_result)
        yolo_bb, yolo_conf = yolo_result
        
        print("Mask Bbox: ", mask_bb, "\nYoloBbox: ", yolo_bb)
        print("Mask conf: ", mask_conf, "\nYolo conf: ", yolo_conf)
       
       # Gather YOLO results
        if yolo_bb is not None:
            ybbx1 = yolo_bb[0]
            ybby1 = yolo_bb[1]
            ybbx2 = yolo_bb[2]
            ybby2 = yolo_bb[3]
            yd = 1
            cy = yolo_conf
        else:
            ybbx1 = 0
            ybby1 = 0
            ybbx2 = 0
            ybby2 = 0
            ybbyd = 0
            yd = 0
            cy = 0
            
            # Gather Mask results
        if (mask_bb is not None) and (len(mask_bb) is not 0):
            mbbx1 = mask_bb[0]
            mbby1 = mask_bb[1]
            mbbx2 = mask_bb[2]
            mbby2 = mask_bb[3]
            md = 1
            cm = mask_conf
        else:
            mbbx1 = 0
            mbby1 = 0
            mbbx2 = 0
            mbby2 = 0
            md = 0
            cm = 0

        # Encode Mask R-CNN results into bitstream
        data_bytes = write_data_string(TYPE=HIO, PN=num_imgs, YD=yd, MD=md, 
                                        YBBX1=ybbx1, YBBY1=ybby1, YBBX2=ybbx2, YBBY2=ybby2,
                                        MBBX1=mbbx1, MBBY1=mbby1, MBBX2=mbbx2, MBBY2=mbby2, 
                                        CY=cy, CM=cm, PIC=img, TEMP=getCPUTemp())
        
        print(data_bytes)      # remove
        
        HIO_data_string = bits2char(data_bytes)
        # Write bitstream to serial comm
        writeData(HIO_data_string)
        num_imgs += 1
        writeStateVariable(STATE_VAR_PATH, "NUMBER_IMAGES", num_imgs)
        
        comm_path = os.getcwd() + '/comms_sent.csv'
        with open(comm_path, 'a', newline='') as comm_csv: # Write to csv file
            writer = csv.writer(comm_csv)
            writer.writerow([HIO_data_string])
        
        #reset timer
        #HIO_timer.stop()
        #HIO_timer.start()
            
    elif img_capture is None:
        print(" IMAGE NOT CAPTURED! Stopping Experiment") #Trying again...")
        #continue
    sleep(0.1)
    # Check S/C state again   
    getSpacecraftState(STATE_VAR_PATH)  

def permission():
    while GPIO.input(ACCESSORY_PWR) == 0:
        sleep(1)
        print("Waiting for permission to run")
    return 1

def main():
    # Check State Variables
    
    # Attempt to read and increment the boot counter
#    boot_counter_str = readStateVariable(STATE_VAR_PATH, "BOOT_COUNTER")
#    try:
#        current_num_boots = int(boot_counter_str[0]) # Get the state variable for BOOT_COUNTER
#    except Exception as e:
#        print("Issue with reading for the state variable: BOOT_COUNTER!")
#        print(e)
#        print(boot_counter_str)
#        current_num_boots = 0
#    current_num_boots += 1
#    writeStateVariable(STATE_VAR_PATH, "BOOT_COUNTER", current_num_boots)

    # Check if we already ran HDD experiment
    read_out = readStateVariable(STATE_VAR_PATH, "HDD_DONE") 
    try:
        hdd_done = read_out[0] # Get the state variable for HDD_DONE
    except TypeError as e:
        print("ISSUE WITH READING THE STATE VARIABLE: HDD_DONE")
        print(e)

    # Check if we already fired burnwire
    read_out = readStateVariable(STATE_VAR_PATH, "BURNWIRE_FIRED") 
    try:
        burnwire_fired = read_out[0] # Get the state variable for BURNWIRE_FIRED
    except TypeError as e:
        print("ISSUE WITH READING THE STATE VARIABLE: BURNWIRE_FIRED")
        print(e)
        
    # Check if we already deployed
    read_out = readStateVariable(STATE_VAR_PATH, "DEPLOYED") 
    try:
        deployed = read_out[0] # Get the state variable for DEPLOYED
    except TypeError as e:
        print("ISSUE WITH READING THE STATE VARIABLE: DEPLOYED")
        print(e)

    # Run system setup
    setup()
    
    #print("hdd done check:", hdd_done)
    #print(type(hdd_done))
    #print(hdd_done == "false")
    
    if hdd_done == "false" and permission():
        HDD_Main()
        #writeStateVariable(STATE_VAR_PATH, "HDD_DONE", "true")
    
    sleep(0.1) # Wait 10 minutes for steady-state
    
    #print("HIO deploy done check:", deployed)
    #print(deployed)
    #print(type(deployed))
    
    if (deployed == "false" or burnwire_fired == "false") and permission():
        HIO_Setup()
        writeStateVariable(STATE_VAR_PATH, "DEPLOYED", "true")
        HIO_Main()
    
    sleep(0.1)
    while 1:
        HDD_Main()
        HIO_Main()
        sleep(20)  #pause between experiments: desired time minus 70 seconds 
    
    # Print final S/C State Variables
    getSpacecraftState(STATE_VAR_PATH)
    
    # Clean up system
    os.system ("sudo killall pigpiod")
    print("Goodbye!")
    sleep(1)

if (__name__ == "__main__"):
    main()