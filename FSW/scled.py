from os import *
from gpiozero import LED
from time import sleep
import RPi.GPIO as GPIO
import threading

class SCLED:
    """
    Driver for the S/C LEDs on the flight board.
    """
    WHITE_LED_PIN = 0
    RGB_PINS = 0
    
    def __init__(self, boardType):
        self.setBoard(boardType)
        print("Board type set to: ", boardType)
        self.initializeLEDs()
        
    def setBoard(board):
        if board == "RPI4":
            WHITE_LED_PIN = 17
            RGB_PINS = [2, 3, 4]

        elif board == "CM4:
            WHITE_LED_PIN = 18
            RGB_PINS = 1
            
    def initializeLEDs():
        GPIO.setup(GPIO.BCM)
        GPIO.setwarnings(False)

        GPIO.setup(WHITE_LED_PIN, GPIO.OUT)
        GPIO.setup(RGB_PINS, GPIO.OUT)
        blink(5)
        
    def blink(numBlinks):
        for i in range(numBlinks):
            print("White LED ON")
            GPIO.output(WHITE_LED_PIN, GPIO.HIGH)
            sleep(1)
            print("White LED OFF")
            GPIO.output(WHITE_LED_PIN, GPIO.LOW)
            
    def destroy():
        #pwmR.stop()
        #pwmG.stop()
        #pwmB.stop()
        GPIO.cleanup()
    
    