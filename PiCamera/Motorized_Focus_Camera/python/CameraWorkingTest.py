import os
import time
import sys

import _thread as thread
from ctypes import *
arducam_vcm =CDLL('./lib/libarducam_vcm.so')

focus_val = 100 # focus value 0-1023

def run_camera(name):
    os.system("raspistill -t 0 --vflip")
if __name__ == "__main__":
    thread.start_new_thread(run_camera, ("run_camera",))
    #vcm init
    arducam_vcm.vcm_init()
    
    arducam_vcm.vcm_write(focus_val)
    
    camera.capture("test_2.jpg")