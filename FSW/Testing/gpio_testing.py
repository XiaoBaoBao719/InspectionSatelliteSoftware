import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)
chan_list = [7, 11, 13]
GPIO.setup(chan_list, GPIO.OUT)
# loop 50 times

for i in range(10):
    GPIO.output(chan_list,GPIO.HIGH)
    time.sleep(1)
    GPIO.output(chan_list, GPIO.LOW)
    time.sleep(1)
GPIO.cleanup()

def set_red():
    GPIO.output(chan_list, GPIO.HIGH)
    