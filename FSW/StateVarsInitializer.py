""" State Variable Initializer
@author: Xiao-Bao Bao
@license: GNU Commons
@version 1.0

@brief State Variable Initializer

Writes initial FSW State Variables to the json file STATE_VARIABLES; which exists on the 
desktop of the payload flight computer. You MUST run this file once before hand off in order
to reset the state variables back to their original config.

Warning: Do not run this script at any time during S/C flight. It will reset all of the 
state variables back to their original values.
"""

import json

def main():

    #file_path = '/home/pi/Desktop/STATE_VARIABLES.json' # For Raspberry Pi environment
    #my_path = '$HOME' + '/InspectionSatCV'
    my_path = '/home/xiaobao/InspectionSatCV/FSW/STATE_VARIABLES.json'

    # Python object
    initial_vars = {"BOOT_COUNTER": 0, 
                    "HARDWARE_ERROR" : "False", 
                    "DEPLOYED" : "False", 
                    "BUTNWIRE_FIRED" : "False",
                    "NUMBER_IMAGES" : 0, 
                    "NUMBER_UNPROCESSED_IMAGES": 0, }

    json_str = json.dumps(initial_vars) # Convert to JSON string
    print(type(json_str))

    with open(my_path, 'w') as json_out: # Write to Json file
        json_out.write(json_str)

    with open(my_path, 'r') as json_out: # Debug statement to read what we just wrote
        print(json.loads(json_out.read()))
    return

if __name__ == "__main__":
    main()