import json

def main():
    #file_path = '/home/pi/Desktop/STATE_VARIABLES.json' # For Raspberry Pi environment
    #my_path = '$HOME' + '/InspectionSatCV'
    my_path = '/home/xiaobao/InspectionSatCV/FSW/STATE_VARIABLES.json'

    # Python object
    initial_vars = {"BOOT_COUNTER": 0, 
                    "HARDWARE_ERROR" : "False", 
                    "DEPLOYED" : "False", 
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