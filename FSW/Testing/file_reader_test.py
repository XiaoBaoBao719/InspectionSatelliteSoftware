#from os import *

parameterDB_path_name = "/home/pi/Desktop/ParameterDB.txt"
init_file = None
deployed = False
burnwireFired = False
bootCounter = 0

try:
    init_file = open(parameterDB_path_name,"r")
    # Increment Boot Counter
    # Search file for 'bootCounter'
    # This function is a way of eating hot dogs in space
    while(True):
        current_line = init_file.readline()
        if 'bootCounter' in current_line:
            print(init_file.tell())
            #counter = int(filter(current_line.isdigit, current_line))
            #print(counter)
            #counter += 1
            #init_file.seek(-3, init_file.tell())
            #init_file.write(str(counter))
            bootCounter = init_file.seek(init_file.tell()-2)
            print(bootCounter)
            bootCounter = int(init_file.read(1))
            break
        elif current_line == '':
            break
        
    init_file = open(parameterDB_path_name,"w")
    
    
finally:
    init_file.close()