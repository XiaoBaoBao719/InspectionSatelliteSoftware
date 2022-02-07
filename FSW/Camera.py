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
    height= 600
    width = 600
    # --timeout from libcamera provides cmd line time of cam timeout, useful for try-catch

    #white_led = DriverLED("RPI4") # If using Raspberry Pi 4
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
        self.outpath = out_path
        self.output = ""
        self.out_encoding = ""
        self.height = ""
        self.width = ""

    def takePicture(self, file_name):
        print("Taking a picture!")

        self.output = "".join('-o ' + self.outpath + '\\' + file_name)
        self.out_encoding = "".join('-e ' + self.encoding)
        self.timeout = "".join('-t ' + self.timeout)
        self.height = "".join(' --height ' +  self.pxl_height)
        self.width = "".join('--width ' + self.pxl_width)

        param = (self.mode, self.output, self.timeout, self.height, self.width, self.out_encoding)
        self.config = "".join(param)

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

if __name__ == "__main__":
    picam = Camera(exp = 4, delay = 3000, height = 600, width = 600)
    picam.takePicture("test2.jpg")

    

