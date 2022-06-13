from os import *
import RPi.GPIO as GPIO
from pathlib import Path, WindowsPath
#from util import *
import sys
import serial
import time
from picam import *


#from . import Camera

orient = 4

def toggleLEDcallback(channel):
    global orient 
    print("Being toggled")
    GPIO.remove_event_detect(18)
    GPIO.output(23, False)
    GPIO.output(24, True)
    # Run Camera Script
    #time.sleep(0.1)
    #run(exposure)
    
    temp1 = 0
#    cam_gain = 0
    iso = 100
    shutter_speed = 8000
    temp2 = 0
    while temp1 < 6:
        shutter_speed = 10
        while shutter_speed <= 700:                #max shutter speed
            time.sleep(0.1)
            run(shutter_speed, iso, o = orient)

#             print("ISO: ",camera.iso, "SS: ",camera.shutter_speed)
            
            temp2 +=1
            shutter_speed += 50                    #max shutter speed/20
            time.sleep(0.1)
        
        temp1 += 1
        iso += 100
        temp2 = 0
            
    orient +=1
    print(orient)
    time.sleep(0.1)
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
    main()

