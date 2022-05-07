from time import sleep
from picamera import PiCamera
import sys
from ctypes import *
import os
import shutil

arducam_vcm =CDLL('./lib/libarducam_vcm.so')
focus_val = 292;
#from Focuser import Focuser

EXPO_TIME = 700
ISO = 600
O_VAL = 0

num = 0

def moveImg(orig, new):
    """ copy file from "orig" directory and
        and save the same file under "new" direcotry
    """
    shutil.copyfile(orig,new)


def run(exposure_time = 700, iso = 600, o = 0):
    global num
    camera = PiCamera()
    arducam_vcm.vcm_init()
    arducam_vcm.vcm_write(focus_val)
    camera.vflip = True 
    camera.shutter_speed = exposure_time
#    camera.awb_gains = gain
    camera.iso = iso
    camera.resolution = (1920,1080)
    camera.framerate = 30
#    camera.start_preview()
#    sleep(1)
#    camera.stop_preview()
    if camera.shutter_speed <100:
        SS4DIR = "0"+str(camera.shutter_speed)
    else:
        SS4DIR = camera.shutter_speed
    if not os.path.isdir("/home/pi/RaspberryPi/Pictures/TT2/ISO/iso:{0}".format(camera.iso)):
        os.mkdir("/home/pi/RaspberryPi/Pictures/TT2/ISO/iso:{0}".format(camera.iso))
    else:
        pass
    
    if not os.path.isdir("/home/pi/RaspberryPi/Pictures/TT2/SS/ss:{0}".format(SS4DIR)):
        os.mkdir("/home/pi/RaspberryPi/Pictures/TT2/SS/ss:{0}".format(SS4DIR))
    else:
        pass
    
    camera.capture("/home/pi/RaspberryPi/Pictures/TT2/ISO/iso:{0}/TT2_iso:{0}_o{1}_ss:{2}.png".format(camera.iso, o, SS4DIR), format = 'png')
    
    
    orig = "/home/pi/RaspberryPi/Pictures/TT2/ISO/iso:{0}/TT2_iso:{0}_o{1}_ss:{2}.png".format(camera.iso, o, SS4DIR)
    new = "/home/pi/RaspberryPi/Pictures/TT2/SS/ss:{2}/TT2_ss:{2}_o{1}_iso:{0}.png".format(camera.iso, o, SS4DIR)
  #    print("ISO: ",camera.iso, "SS: ",camera.shutter_speed)
  
    moveImg(orig,new)
    num+=1
    camera.close()
    
def getIsoVal(iso):
    return iso

def getNumPicsTaken(num):
    return num
    
if __name__ == "__main__":
    run()
    
    