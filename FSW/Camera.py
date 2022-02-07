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

out_path = "\home\pi\Desktop"
#HEIGHT = 0
#WIDTH = 0

class Camera():

    exposure_time = 0
    delay = 0
    shutter_speed = 0
    gain = 0
    pxl_height = 0
    pxl_width = 0
    timeout = 0
    config = ''
    mode = ''
    encoding = ''
    flush = ' --wrap '
    preview_mode = ''
    # --timeout from libcamera provides cmd line time of cam timeout, useful for try-catch

    white_led = DriverLED("RPI4") # If using Raspberry Pi 4
    #white_led = DriverLED("CM4")  # If using Computer Module 4

    def __init__(self, exp, _delay, height, width):
        self.pxl_height = height
        self.pxl_width = width
        self.exposure_time = exp
        self.delay = _delay
        self.timeout = 5000 # 5 second MAX for timeout
        self.encoding = 'jpg'
        self.mode = 'libcamera-still '
        self.preview_mode = ' --nopreview ' # used only for debugging

    def takePicture(self, file_name):
        print("Taking a picture!")

        output = str.join(' -o ', self.outpath + '\\' + file_name)
        out_encoding = str.join(' -r ', self.encoding)
        timeout = str.join(' -t ', self.timeout)
        height = str.join(' --height ', self.pxl_height)
        width = str.join(' --width ', self.pxl_width)
        
        self.config = str.join(self.mode, output, timeout, 
                                height, width, out_encoding, 
                                self.flush, self.preview_mode)
        try:
            system(self.config)
        except Exception:
            print("Failed to take picture due to raised exception: ", system.exec_info()[2])
            return False
        #system('libcamera-jpeg -o handrail-input.jpg -t 5000 --width 800 --height 600')
        return True

    def getNumPicsTake(self):
        pass

    # def turnOnLight(self):
    #     pass

    # def turnOffLight(self):
    #     pass

    

