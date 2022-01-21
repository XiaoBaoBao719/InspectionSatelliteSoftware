from FSW.Main import EXPOSURE_TIME
from os import *
from RPi.GPIO import GPIO
from pathlib import Path, WindowsPath

import serial
import threading
import DriverLED

# =====================================
# ==         GLOBAL VARS         ==
# =====================================

outputPath = ""
HEIGHT = 0
WIDTH = 0

class Camera():

    exposure_time = 0
    delay = 0
    shutter_speed = 0
    # --timeout from libcamera provides cmd line time of cam timeout, useful for try-catch

    white_led = DriverLED("RPI4") # If using Raspberry Pi 4
    #white_led = DriverLED("CM4")  # If using Computer Module 4

    def __init__(self, exp, delay, height, width, ):
        pass

    def takePicture(self):
        pass

    def getNumPicsTake(self):
        pass

    def turnOnLight(self):
        pass

    def turnOffLight(self):
        pass

    

