import RPi.GPIO as GPIO
import os
from Main import readStateVariable
from Main import writeStateVariable
from Main import initializeComputer

initializeComputer()
STATE_VAR_NAME = "STATE_VARIABLES.json"
STATE_VAR_PATH = os.getcwd() + '/' + STATE_VAR_NAME


boot_counter_str = readStateVariable(STATE_VAR_PATH, "BOOT_COUNTER")
try:
        current_num_boots = int(boot_counter_str[0]) # Get the state variable for BOOT_COUNTER
except Exception as e:
    print("Issue with reading for the state variable: BOOT_COUNTER! on startup")
    print(e)
    print(boot_counter_str)
    current_num_boots = 0
current_num_boots += 1
writeStateVariable(STATE_VAR_PATH, "BOOT_COUNTER", current_num_boots)
if current_num_boots > 1:
    os.system("sudo -E python3 Main.py")

else:
    print("Running Initializer")   #debug only
    os.system("python3 Initializer.py")    #debug only
    writeStateVariable(STATE_VAR_PATH, "BOOT_COUNTER", current_num_boots)
    print("Starting Main")
    os.system("sudo -E python3 Main.py")
