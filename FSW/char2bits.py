import numpy as np

def char2bits(chars):
    """
    chars is a string containing data, example "I?d4C?31peqa27V"
    return bit array
    sorry I'm so rusty in python
    """
    n = len(chars)   #number of characters
    bits = np.zeros((6*n,), dtype=int)
    key = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789!?"
    count = 0
    for ch in chars:
        m = key.find(ch)
        bits[0 + 6*count] = (m % 2)
        bits[1 + 6*count] = (m % 4)>1.5
        bits[2 + 6*count] = (m % 8)>3.5
        bits[3 + 6*count] = (m % 16)>7.5
        bits[4 + 6*count] = (m % 32)>15.5
        bits[5 + 6*count] = (m % 64)>31.5
        count += 1
    
    return bits
