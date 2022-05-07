from os import *
import RPi.GPIO as GPIO
from pathlib import Path, WindowsPath
#from util import *
import sys
import serial
import time
from Camera import Camera

#picam = Camera(exp = 8000, timeout = 5000, gain = 10, delay = 3000, height = 1080, width = 1920)
#cam_gain = 10

def toggleLEDcallback(channel):
    print("Being toggled")
    GPIO.remove_event_detect(18)
    GPIO.output(23, False)
    GPIO.output(24, True)
    # Run Camera Script
    temp1 = 0
    cam_gain = 10
    shutter_speed = 8000            #  (1000, 11000. 2000)
    temp2 = 0
    while temp1 < 5:
        shutter_speed = 8000
        while temp2 < 6:
            time.sleep(0.1)
            print(picam.getGainVal())
            picam.takePicture("test{num}_".format(num = picam.getNumPicsTake()), gain = cam_gain, exposure_time = shutter_speed, timeout = 5000)
            print(picam.getGainVal())
                            
            temp2 += 1
            shutter_speed += 1000
            time.sleep(0.01)
        
        temp1 += 1
        cam_gain += 10
        temp2 = 0
    
    cam_gain = 10
    
    
    time.sleep(1)
    GPIO.output(23, True)
    GPIO.output(24, False)
    GPIO.add_event_detect(18, GPIO.FALLING, callback=toggleLEDcallback, bouncetime=1000)
        
def main(): 
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    # Setup GPIO 17 as a PUSH BUTTON INTERRUPT pull-up resistor 
    # setup GPIO 23 as a DEBUGGING LED that tells us that the camera is ready to take another image when blinking
    GPIO.setup(18, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(23, GPIO.OUT)

    # Setup GPIO 24 as the WHITE LED from the lighting board
    # Setup GPIO 25 as the PHOTODIODE that records the lighting information
    GPIO.setup(24, GPIO.OUT)
    GPIO.setup(25, GPIO.IN)

    GPIO.add_event_detect(18, GPIO.FALLING, callback=toggleLEDcallback, bouncetime=1000)
    GPIO.output(23, True)
    GPIO.output(24, True)
    try:
        while True:
            if GPIO.input(18):
                #print("ON")
                #GPIO.output(23, True)
                pass
            else:
                #print("OFF")
                #GPIO.output(23, False)
                pass
        print(GPIO.output(23))
        time.sleep(0.1)
    finally: # Used to handle keyboard interrupts
        GPIO.output(23, False)
        GPIO.output(24, False)
        GPIO.cleanup()

    GPIO.cleanup() # Clean up normally
    pass

if __name__ == "__main__":
    
    picam = Camera(height = 600, width = 600)
    main()

