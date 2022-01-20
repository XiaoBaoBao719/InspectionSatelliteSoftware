import RPi.GPIO as GPIO
import time
import random

pins = [11, 13, 15] # R = 11, G = 12, B = 13

def setup():
    global pwmR, pwmG, pwmB
    GPIO.setmode(GPIO.BOARD)
    #for i in pins: #iterate on the RGB pins
    GPIO.setup(pins, GPIO.OUT)
    GPIO.output(pins, GPIO.HIGH)
    pwmR = GPIO.PWM(pins[0], 2000)  # set each PWM pin to 2 KHz
    pwmG = GPIO.PWM(pins[1], 2000)
    pwmB = GPIO.PWM(pins[2], 2000)
    pwmR.start(0)   # initially set to 0 duty cycle
    pwmG.start(0)
    pwmB.start(0)
    
def setColor(r,g,b): # 0~100 values
    pwmR.ChangeDutyCycle(r)
    pwmG.ChangeDutyCycle(g)
    pwmB.ChangeDutyCycle(b)
    
def displayColors():
    setColor(100, 0, 0) #   red color
    time.sleep(1)   # 1s
    setColor(0, 100, 0) # green
    time.sleep(1)   # 1s
    setColor(0, 0, 100) # blue
    time.sleep(1)   # 1s
    setColor(100, 100, 0) # yellow
    time.sleep(1)   # 1s
    setColor(0, 100, 100) # cyan
    time.sleep(1)   # 1s
    setColor(100, 0, 100) # magenta
    time.sleep(1)   # 1s
    setColor(50, 0, 0) # maroon
    time.sleep(1)   # 1s
    setColor(50, 0, 50) # purple
    time.sleep(1)   # 1s
    setColor(0, 0, 50) # navy
    time.sleep(1)   # 1s    
    
def destroy():
    pwmR.stop()
    pwmG.stop()
    pwmB.stop()
    GPIO.cleanup()
    
if __name__=='__main__':
    setup()
    displayColors()
    destroy()