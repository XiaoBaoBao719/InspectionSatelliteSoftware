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
#from loguru import RotationFunction, logger
from pathlib import Path, WindowsPath

import numpy as np
import math
#import loguru
import RPi.GPIO as GPIO
import serial
import threading
import json

<<<<<<< HEAD
# Import JSC Payload and packages
from JSC_FLIGHT_HDD_EXP import HDD_ccw_drive, HDD_cw_drive, HDD_stop, HDD_print
from serial.serialutil import SerialException
=======
import Burnwire
import DriverLED as led
import Camera as cam
>>>>>>> 45212150f829b6be4a00061da4ea0e913af002dc

from DetectronPredictor import *
from YoloPredictor import run


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
current_time = 0
parameterDB_path_name = '\home\pi\Desktop\ParameterDB'
state_vars_path = '\home\pi\Desktop\''
state_vars_dict = {}
#init_file = ""
#deployed = False
#burnwireFired = False


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
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    #GPIO.setup(LED_PIN, GPIO.OUT)
    startTimer()

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
    ser: Serial
        Serial object handles UART comms protocol with S/C
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
    detectron_results_lit = DetectronPredictor.detect(source)
    yolo_results_lit = YoloPredictor.detect(source)

    camLightOff()
    detectron_results_dark = DetectronPredictor.detect(source)
    yolo_results_dark = YoloPredictor.detect(source)

    return [{'Light': (detectron_results_lit, yolo_results_lit)} ,
            {'Dark':(detectron_results_dark, yolo_results_dark)}]
    
    
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

<<<<<<< HEAD
def doHDD():
    # [w_x0, w_y0, w_z0, w_x1, w_y1, w_z1, w_xf, w_yf, w_zf, HDD_current_f] = HDD_ccw_drive(sleep_time, delta)
    # [w_x0, w_y0, w_z0, w_x1, w_y1, w_z1, w_xf, w_yf, w_zf, HDD_current_f] = HDD_cw_drive(sleep_time, delta)
    # HDD_stop()   uncomment when testing!
    flashled(5)
    # [w_x0, w_y0, w_z0, w_x1, w_y1, w_z1, w_xf, w_yf, w_zf, HDD_current_f] = [-1.0, -2.0, -3.0, 1.0, 2.0, 3.0, -1.0, -2.0, -3.0, 0.05]
    result = [-1.0, -2.0, -3.0, 1.0, 2.0, 3.0, -1.0, -2.0, -3.0, 0.05]
    HDD_print(result)
    return result
=======
    ser.flush()
    return packetStatus
>>>>>>> 45212150f829b6be4a00061da4ea0e913af002dc

def setup(self):
    """ Performs initial bootup sequence once and deploys the payload mechanism
        Checks status of components by writing to a text doc
    Parameters
    ----------
    val: 
    """
    #initializeComputer()
    #serialSetup()
    #initializeBurnwire()
    
    # try-catch guarentees that the file is properly closed even when an exception is raised
    # that could prevent us from closing the file
    try:
        self.init_file = open(parameterDB_path_name,mode='w',encoding='utf-8')
        # Increment Boot Counter
        # Search file for 'bootCounter'
        while(True):
            current_line = self.init_file.readline()
            if 'bootCounter' in current_line:
                print(self.init_file.tell())
                counter = int(filter(str.isdigit, current_line))
                counter += 1
                self.init_file.seek(-3, self.init_file.tell())
                self.init_file.write(str(counter))
                break
            elif current_line == '':
                break
    finally:
        self.init_file.close()
    # Check the deploy flag on init file

    try:
        self.init_file = open(parameterDB_path_name,mode='r',encoding='utf-8')
        # Search file for 'deployedFlag'
        while(True):
            current_line = self.init_file.readline()
            if 'deployedFlag' in current_line:
                if 'False' in current_line:
                    print(self.init_file.tell())
                    print("Deployed Flag is False")
                    self.deployed = False
                    break
                elif 'True' in current_line:
                    print(self.init_file.tell())
                    print("Deployed Flag is True")
                    self.deployed = True
                    break
            elif current_line == '':
                break
    finally:
        self.init_file.close()
    
    if self.deployed is False:
        # Fire the Burnwire
        burnwire = Burnwire(2, 5000, 0)
        burnwire.getBurnwireStatus()
        burnwire.burn()
        burnwire.destroy()

        # Set burnwireFired to true
        try:
            self.init_file = open(parameterDB_path_name,mode='w',encoding='utf-8')
            # Search file for 'deployedFlag'
            while(True):
                current_line = self.init_file.readline()
                if 'burnwireFired' in current_line:
                    if 'False' in current_line:
                        print(self.init_file.tell())
                        print("Burnwire Flag was False") #TODO: Need a better way of doing this
                        self.init_file.write(str.replace('False','True'))
                        self.burnwireFired = True
                        break
                    elif 'True' in current_line:
                        print(self.init_file.tell())
                        print("Burnwire Flag was True")
                        self.deployed = True
                        break
                elif current_line == '':
                    break
        finally:
         self.init_file.close()

    else:
        # Initialize and define the systems
        pass
                
    if not verifySystem():
        reboot()
<<<<<<< HEAD
    
    print("Health Check Complete. \n Standing by")
    #flashled(20)
    sleep(5)
    
TOTAL_HDD_EXPERIMENTS = 2 # number experiments to perform
TIME_PER_HDD = 10 # 5 mins between each experiment

def HDD_Main():
    # Start HDD Experiment
    print("Initiating HDD experiment...")
    #flashled(10)
    sleep(5)
    HDD_results = []
    HDD_timer = FSWTimer()
    HDD_timer.start()

    last_results = []
    curr_results = []
    num_runs = 0
    hdd_bytes = ""
    
    while num_runs <= TOTAL_HDD_EXPERIMENTS:
        if (HDD_timer.elapsed_time() % TIME_PER_HDD >= 0):
            print("Running HDD experiment: ", num_runs)
            print(f"Elapsed time: {HDD_timer.elapsed_time():0.4f} seconds")
            curr_results = doHDD()
            HDD_results.append(curr_results)
            
            print("Run successful!")
            # sleep(TIME_PER_HDD) # Wait for system to settle after 5 mins
            wxa = curr_results[0]
            wxb = curr_results[1]
            wxc = curr_results[2]
            wya = curr_results[3]
            wyb = curr_results[4]
            wyc = curr_results[5]
            wza = curr_results[6]
            wzb = curr_results[7]
            wzc = curr_results[8]
            cd = curr_results[9]

            # Write HDD results to UCD Data buffer
            print("Writing HDD data to buffer...")
            hdd_bytes = write_data_string(TYPE=HDD, XN=num_runs, WXA=wxa, WXB=wxb, WXC=wxc, 
                                            WYA=wya, WYB=wyb, WYC=wyc, WZA=wza, WZB=wzb, WZC=wzc,
                                            CD=cd, TEMP=getCPUTemp())
            writeData(hdd_bytes)
        # Update
        num_runs += 1
        
    print(f"Elapsed HDD time: {HDD_timer.elapsed_time():0.4f} seconds")
    HDD_timer.stop()
    
    pass
=======
>>>>>>> 45212150f829b6be4a00061da4ea0e913af002dc

    pass


<<<<<<< HEAD
def HIO_Setup():
    getSpacecraftState(STATE_VAR_PATH)
    # If deployed is FALSE, create a Burnwire() object and invoke the burn function
    # Runs .burn() for both pins at 5000 Hz for 1 second
    read_out = readStateVariable(STATE_VAR_PATH, "BURNWIRE_FIRED") # Check for the state variable for DEPLOYED
    
    print(read_out)
    
    try:
        burn_fired = read_out[0] # Get the state variable for Deployed
    except TypeError as e:
        print("ISSUE WITH READING THE STATE VARIABLE: DEPLOYED")
        print(e)
    
    print("Checking deployed state: ", deployed)
    print("burnwire fired? ", type(burn_fired))
    
    if burn_fired == False:
        # SETUP AND DEPLOY BURNWIRE 
        burnwire = Burnwire(NUM_BURNWIRES)
        burnwire.getBurnwireStatus()
        sleep(2)
        burn_channels = [1, 2]

        # Write to state variable in case of burnwire forced reboot event
        writeStateVariable(STATE_VAR_PATH, "BURNWIRE_FIRED", True)
        writeStateVariable(STATE_VAR_PATH, "DEPLOYED", True)
        burn_result = burnwire.burn(burn_channels)
        
        # If we successfully deployed nominally
        """
        if burn_result:
            print("Burn successful!")
            writeStateVariable(STATE_VAR_PATH, "DEPLOYED", burn_result)
        else:
            print("Burn attempt failed!")
            writeStateVariable(STATE_VAR_PATH, "DEPLOYED", burn_result)
        """
    getSpacecraftState(STATE_VAR_PATH)

def HIO_Main():
    
    # image_path = picam.getCapturePath()
    
    print("Initiating HIO Experiment...")
    sleep(5)
    HIO_timer = FSWTimer()
    HIO_timer.start()
    
    num_imgs = 0

    while num_imgs <= MAX_IMGS-1:
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
            if img_capture is not img:
                # Debugging: show the image
                # cv.imshow("debug img", img)
                # cv.waitKey(0)
                # cv.destroyAllWindows()

                # Perform model inference on the returned path from the model              
                det_dict = Inference_Mask(img, INFERENCE_THRESHOLD)
                
                ### DEBUGGING TABULATED DETECTION RESULTS ###
                displayResults(det_dict)
                #det_dict = {0: {'bbox': [371.8197, 312.28333, 1514.4973, 665.65515], 'conf': 0.99867}, 1: {'bbox': [998.0732, 295.11374, 1515.9216, 582.3098], 'conf': 0.07558}}

                _bb, _conf = getBestResults(det_dict)
                
                print(_bb)
                
                if _bb is not None:
                    x1 = _bb[0]
                    y1 = _bb[1]
                    x2 = _bb[2]
                    y2 = _bb[3]
                    yd = 1
                    cy = _conf
                else:
                    x1 = 0
                    y1 = 0
                    x2 = 0
                    y2 = 0
                    yd = 0
                    cy = 0  

                # Encode YOLO results into bitstream
                data_bytes = write_data_string(TYPE=HIO, PN=num_imgs, YD=yd, 
                                                YBBX1=x1, YBBY1=y1, YBBX2=x2, YBBY2=y2, 
                                                CY=cy, PIC=img, TEMP=getCPUTemp())

                # Write bitstream to serial comm
                writeData(data_bytes)
                num_imgs += 1
                writeStateVariable(STATE_VAR_PATH, "NUMBER_IMAGES", num_imgs)
                
            elif img_capture is None:
                print(" IMAGE NOT CAPTURED! Trying again...")
                continue
        sleep(1)
        
    # Check S/C state again   
    getSpacecraftState(STATE_VAR_PATH)  
=======
def main():
    """ Main loop
    """
    setup()
>>>>>>> 45212150f829b6be4a00061da4ea0e913af002dc

    print(timer)

    while timer % timeInterval != 0:
        """ Performs an inference every six minutes """
        imgPath = takePicture(EXPOSURE_TIME, DELAY, WIDTH, HEIGHT)
        results = performInference(imgPath)
        writePayloadData(results)
        #TODO: must be able to read from Rx and implement parity bit outcome
        pass
    ser.__exit__()    

<<<<<<< HEAD
    HDD_Main()
    sleep(5) # Wait 10 minutes for steady-state
    #HIO_Setup()
    sleep(5)
    #HIO_Main()
    
    # Clean up system
    #ser.close()
    os.system ("sudo killall pigpiod")
    print("Goodbye!")
    sleep(2)
    #ser.__exit__()
    
=======
if __name__ == "__main__":
    main()
>>>>>>> 45212150f829b6be4a00061da4ea0e913af002dc
