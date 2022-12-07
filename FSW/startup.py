import RPi.GPIO as GPIO
import os
from Main import readStateVariable


STATE_VAR_NAME = "STATE_VARIABLES.json"
STATE_VAR_PATH = os.getcwd() + '/' + STATE_VAR_NAME

HDD_POWERED_PIN = 26 #GPIO 26
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(HDD_POWERED_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

boot_counter_str = readStateVariable(STATE_VAR_PATH, "BOOT_COUNTER")
try:
        current_num_boots = int(boot_counter_str[0]) # Get the state variable for BOOT_COUNTER
except Exception as e:
    print("Issue with reading for the state variable: BOOT_COUNTER! on startup")
    print(e)
    print(boot_counter_str)
    current_num_boots = 0
current_num_boots += 1
if current_num_boots != 0:
    os.system("sudo -E python3 Main.py")

while 1:
    if GPIO.input(HDD_POWERED_PIN):
        print("HDD powered on")
        print("Running Initializer")   #debug only
        os.system("python3 Initializer.py")    #debug only
        print("Starting Main")
        os.system("sudo -E python3 Main.py")
        break
