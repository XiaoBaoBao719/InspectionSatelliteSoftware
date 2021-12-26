import RPi.GPIO as GPIO
import time

PHOTODIODE_PIN = 9
INPUT_PIN = 10
BURN_PIN_1 = 11
BURN_PIN_2 = 12

burnChanList = (BURN_PIN_1, BURN_PIN_2)

GPIO.setmode(GPIO.BOARD)
GPIO.setup(BURN_PIN_1, GPIO.OUT, initial = GPIO.LOW)
GPIO.setup(BURN_PIN_2, GPIO.OUT, initial = GPIO.LOW)
GPIO.setup(INPUT_PIN, GPIO.IN)
GPIO.setup(PHOTODIODE_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

# Immediately perform burnwire deploy for 5 seconds
GPIO.output(burnChanList, GPIO.HIGH)
prtStatus = GPIO.output(burnChanList, not GPIO.input(burnChanList))
time.sleep(5) # Wait 5 seconds
# Shutdown burnwire!
GPIO.output(burnChanList, GPIO.LOW)
prtStatus += GPIO.output(burnChanList, not GPIO.input(burnChanList)) # Report to sys diagnostic
time.sleep(2) # Wait 2 seconds

if burnwireNotDeployed():
    prtStatus += "Burnwire not deployed, trying again"
    # Initialize timer to zero, run for 20 mins


    # Check if burnwire deploy successful 
    while GPIO.input(PHOTODIODE_PIN) == GPIO.low and timer < 20:
        # Tell the Main class to not run the Camera Science packet
        prtStatus += "Waiting for confirmation of successful burnwire"


GPIO.cleanup()
