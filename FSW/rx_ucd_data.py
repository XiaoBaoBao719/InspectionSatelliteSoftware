#read and write data to .JSON file stored on UP board
#AZ, HRVIP, May 12 2022

import numpy as np
import json
import os.path
import time
import serial
from read_data_string import read_data_string
from char2bits import char2bits

def rx_ucd_data(new_data):
    """
    Given: string of new data, "abcdefabcdefabc"
    Return: nothing (just writes to file)
    sorry I'm so rusty in python
    """

#    #check if data file exists yet (if first data string, it will not?)
#    file_path = 'idk'
#    #if data file does not exist, create it
#
#    #read existing data and add new data to list
#    data = {} #temp Python dict to hold data
#    try:
#        with open(file_path, 'r') as json_out:
#            data = json.loads(json_out.read()) #converts json file to dict
#    except Exception as e:
#        print("Error reading json data file")
#        print("Exception: ", e)
#    num = max(data['order'])
#    data['order'] = np.append(data['order'], num+1)


    if not os.path.exists('sampleZ.json'):
        #create JSON file
        #order = (0)
        data = new_data

        datafile = {
            #"order": [order],
            "data": [data]
        }

        with open("sampleZ.json", "w") as outfile:
            json.dump(datafile, outfile)
        return
 
    datafile = json.load(open('sampleZ.json',))

    #append new data
    num = len(datafile["data"])
    datafile["data"].append(new_data)
    #datafile["order"].append(num)
    num += 1
    #datafile["order"] = [range(num)]
    #determine first item on list, HIO (1) or HDD (0)

    first_line = datafile["data"][0]
    first_bits = char2bits(first_line)
    TYPE = first_bits[0]

    #count number of HIO (B) and HDD(A) data strings
    n_A = 0
    n_B = 0
    L_A = [] #list to hold only HDD data
    L_B = [] #list to hold only HIO data
    for d in datafile["data"]:
        temp = char2bits(d)
        if temp[0]: #HIO
            n_B += 1
            L_B.append(d)
        else:
            n_A += 1
            L_A.append(d)
    
    #determine confidence
    con = np.zeros((n_B,), dtype=float)
    i = 0
    for d in L_B:
        temp = read_data_string(char2bits(d))
        con[i] = temp[12]
        i += 1

    #sort based on confidence
    #np.disp(con)
    ind = np.argsort(con)
    ind = np.flip(ind)
    #np.disp(ind)
    L_Bsort = []
    for i in range(n_B):
        L_Bsort.append(L_B[ind[i]])

    #merge the two lists back together
    i_A = 0
    i_B = 0
    #L_C = []
    for i in range(num):
        if TYPE:
            TYPE = False
            if i_B < n_B:
                datafile["data"][i] = L_Bsort[i_B]
                i_B += 1
            else:
                datafile["data"][i] = L_A[i_A]
                i_A += 1
        else:
            TYPE = True
            if i_A < n_A:
                datafile["data"][i] = L_A[i_A]
                i_A += 1
            else:
                datafile["data"][i] = L_Bsort[i_B]
                i_B += 1
    
    #write new list to json file
    #np.disp(datafile["data"])
    with open("sampleZ.json", "w") as outfile:
        json.dump(datafile, outfile)
