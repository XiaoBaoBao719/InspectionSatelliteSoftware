import cv2 #sudo apt-get install python-opencv
import numpy as py
import os
import time
import shutil
from ctypes import *
# load arducam shared object file
arducam_vcm= CDLL('./lib/libarducam_vcm.so')
try:
    import picamera
    from picamera.array import PiRGBArray
except:
    sys.exit(0)
    
def run_camera():
    # set desired focus value from 0-1023
    focus_val = 100
    # vcm init
    arducam_vcm.vcm_init()
    # open camera
    camera = picamera.PiCamera()
    # open camera preview
    camera.start_preview()
    # set camera resolution to 640x480(Small resolution for faster speeds.)
    camera.resolution = (640, 480)
    time.sleep(0.1)
    camera.shutter_speed=30000
    
    arducam_vcm.vcm_write(focus_val)
  
    # set camera rotation
    camera.rotation = 180
    # set image view time
    time.sleep(1)
    # set camera resolution to 2592x1944
    camera.resolution = (1920,1080)
    # current date and time
    datetime_str = str(time.strftime("%d_%m_%Y-%H_%M_%S"))
    # image name
    image_name = datetime_str + ".jpg"
    # save image to file
    camera.capture(image_name)
    # end image visual
    camera.stop_preview()
    camera.close()
    # move images to desired location
    orig_path = os.path.abspath(image_name)
    end_path = os.path.abspath("Captured_Images")
    shutil.move(orig_path, end_path)
    entire_path = end_path + "/" + image_name
    return entire_path

if __name__ == "__main__":
    run_camera()
    
    