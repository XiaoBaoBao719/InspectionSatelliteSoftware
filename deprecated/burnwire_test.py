#BW test
import os
import time
import sys
import RPi.GPIO as GPIO
from time import sleep

Burn = 12

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(Burn,GPIO.OUT)

GPIO.output(Burn, GPIO.HIGH)
sleep(5)
GPIO.output(Burn, GPIO.LOW)
