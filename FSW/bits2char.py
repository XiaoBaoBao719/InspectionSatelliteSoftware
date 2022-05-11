import numpy as np
import math

def bits2char(bits):
    """
    Given bits string (np array of 0s and 1s)
    return chars, a string containing data, example "I?d4C?31peqa27V"
    
    sorry I'm so rusty in python
    """
    
    # pad if necessary
    n = 6*math.ceil(len(bits)/6)
    bit = np.zeros((n,), dtype=int)
    bit[0:len(bits)] = bits

    #create data string
    data_out = "a"
    data_out = list(data_out * int(n/6))
    n2 = len(data_out)

    #encode data
    key = list("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789!?")
    for i in range(n2):
        b = bit[6*i : 6*i+7]
        num = b[0] + 2*b[1] + 4*b[2] + 8*b[3] + 16*b[4] + 32*b[5]
        data_out[i] = key[num]
    data_out = "".join(data_out)
    return data_out
