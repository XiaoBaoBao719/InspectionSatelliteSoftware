import numpy as np
import math
from PIL import Image
import json
import time
import serial
import os.path
import mraa

#create serial object and start listening (never closes port)
ser = serial.Serial(
        port='/dev/ttyS4', #port on UP board, pins # 8 (TX) and 10 (RX). Don't forget to connect GND
        baudrate=19200,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=1
    )

#Setup GPIO Pin
gpio_UCD_activate = mraa.Gpio(23)
gpio_UCD_activate.dir(mraa.DIR_OUT)
gpio_UCD_activate.write(0)

#Send activation signal
print("Sending activation signal in 3 seconds")
time.sleep(3)
gpio_UCD_activate.write(1)
print('GPIO HIGH')
time.sleep(0.4)
print('Now listening on UART Line')

def check_ucd_data():    
    #check for data that has entered buffer
    #TODO: implement handling for null characters
    #TODO: implement starting special characters to indicate beginning of new message (comma?)
    if ser.inWaiting() > 14:
        message = ser.read(15)
        print(message)  

        #stripped, replace, read new bytes - see Sam's code
        stripped = message.replace(b'\xf4',b'')
        stripped = stripped.replace(b'\x00',b'')
        len_stripped = len(stripped)
        to_read=0
        if len_stripped < 15:
            to_read = 15-len_stripped
            message = stripped+ser.read(to_read)

        rx_ucd_data(message.decode('utf-8'))
        data = read_data_string(char2bits(message.decode('utf-8')))
        # display incoming data
        if data[0]:
        	print("Packet type: HIO")
        	print("Photo number: "+str(data[1]))
        	print("YOLO detected? "+str(data[2]))
        	print("MASK detected? "+str(data[3]))
        	print("YBB X1: "+str(data[4]))
        	print("YBB Y1: "+str(data[5]))
        	print("YBB X2: "+str(data[6]))
        	print("YBB Y2: "+str(data[7]))
        	print("MBB X1: "+str(data[8]))
        	print("MBB Y1: "+str(data[9]))
        	print("MBB X2: "+str(data[10]))
        	print("MBB Y2: "+str(data[11]))
        	print("YOLO confidence: "+str(data[12]))
        	print("MASK confidence: "+str(data[13]))
        	print("Q30: "+str(data[14]))
        	print("Q50: "+str(data[15]))
        	print("Q70: "+str(data[16]))
        	print("Q90: "+str(data[17]))
        	print("Temperature: "+str(data[21]))
        else:
        	print("Packet type: HDD")
        	print("Exp #: "+str(data[1]))
        	print("WXA: "+str(data[2]))
        	print("WXB: "+str(data[3]))
        	print("WXC: "+str(data[4]))
        	print("WYA: "+str(data[5]))
        	print("WYB: "+str(data[6]))
        	print("WYC: "+str(data[7]))
        	print("WZA: "+str(data[8]))
        	print("WZB: "+str(data[9]))
        	print("WZC: "+str(data[10]))
        	print("CD: "+str(data[11]))
        	print("Temp: "+str(data[12]))
        	


def rx_ucd_data(new_data):
    """
    Given: string of new data, "abcdefabcdefabc"
    Return: nothing (just writes to .json file)
    sorry I'm so rusty in python
    """

    #check if data file exists yet (if first data string, it will not?)
    if not os.path.exists('ucd_data.json'):
        #create JSON file
        #order = (0)
        data = new_data

        datafile = {
            #"order": [order],
            "data": [data]
        }

        with open("ucd_data.json", "w") as outfile:
            json.dump(datafile, outfile)
        return
 
    datafile = json.load(open('ucd_data.json',))

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



def write_data_string(TYPE=0,PN=0,YD=0,MD=0,YBBX1=0,YBBY1=0,YBBX2=0,YBBY2=0,
                      MBBX1=0,MBBY1=0,MBBX2=0,MBBY2=0,CY=0,CM=0,PIC=np.array([[0]]),
                      XN=0,WXA=0,WXB=0,WXC=0,WYA=0,WYB=0,WYC=0,WZA=0,WZB=0,WZC=0,CD=0,TEMP=0):
    """
    Return: bit string (np array of type ints) of the data (LSB first)
    Given:
    TYPE: boolean, whether HDD (0) or HIO (1)
    PN: photo number, will wrap around after > 31
    YD: boolean, whether YOLO detected handrail
    MD: boolean, whether MASK detected handrail
    YBBX1: YOLO bounding box coordinate, x1
    YBBY1: YOLO bounding box coordinate, y1
    YBBX2: YOLO bounding box coordinate, x2
    YBBY2: YOLO bounding box coordinate, y2
    MBBX1: MASK bounding box coordinate, x1
    MBBY1: MASK bounding box coordinate, y1
    MBBX2: MASK bounding box coordinate, x2
    MBBY2: MASK bounding box coordinate, y2
    CY: confidence of YOLO
    CM: confidence of MASK
    PIC: the image taken by the camera
    XN: experiment number for the HDD payload (0-31)
    WXA: rotation rate, x axis, before experiment, deg/s
    WXB: rotation rate, x axis, during experiment, deg/s
    WXC: rotation rate, x axis, after  experiment, deg/s
    WYA: rotation rate, y axis, before experiment, deg/s
    WYB: rotation rate, y axis, during experiment, deg/s
    WYC: rotation rate, y axis, after  experiment, deg/s
    WZA: rotation rate, z axis, before experiment, deg/s
    WZB: rotation rate, z axis, during experiment, deg/s
    WZC: rotation rate, z axis, after  experiment, deg/s
    CD: current draw of the HDD, in mA
    TEMP: temperature reported by the flight computer, *C

    """
    
    #generate bit string
    bits = np.zeros((90,), dtype=int)
    
    bits[0] = TYPE

    # switch depending on whether HIO or HDD data
    #first, HIO data:
    if TYPE:
        m, n, d = PIC.shape
        #PIC_G = PIC.convert('L')
        PIC_G = PIC[:,:,0] * PIC[:,:,0] + PIC[:,:,1] * PIC[:,:,1] + PIC[:,:,2] * PIC[:,:,2]
        PIX = PIC[656, 1159]  #special pixel, right in the middle of the handrail

        #encode photo number
        bits[1] = (PN %  2)     
        bits[2] = (PN %  4)> 1.5
        bits[3] = (PN %  8)> 3.5
        bits[4] = (PN % 16)> 7.5
        bits[5] = (PN % 32)>15.5

        bits[6] = YD  
        bits[7] = MD

        #make sure bounding box is within boundaries
        YBBX1 = min(m, YBBX1) 
        YBBX1 = max(1, YBBX1)
        YBBY1 = min(n, YBBY1) 
        YBBY1 = max(1, YBBY1)
        YBBX2 = min(m, YBBX2) 
        YBBX2 = max(1, YBBX2)
        YBBY2 = min(n, YBBY2) 
        YBBY2 = max(1, YBBY2)
        MBBX1 = min(m, MBBX1) 
        MBBX1 = max(1, MBBX1)
        MBBY1 = min(n, MBBY1) 
        MBBY1 = max(1, MBBY1)
        MBBX2 = min(m, MBBX2) 
        MBBX2 = max(1, MBBX2)
        MBBY2 = min(n, MBBY2) 
        MBBY2 = max(1, MBBY2)

        bb = np.zeros(8, dtype=int)
        bb[0] = math.floor(16*(YBBX1-1)/m)
        bb[1] = math.floor(16*(YBBY1-1)/n)
        bb[2] = math.floor(16*(YBBX2-1)/m)
        bb[3] = math.floor(16*(YBBY2-1)/n)
        bb[4] = math.floor(16*(MBBX1-1)/m)
        bb[5] = math.floor(16*(MBBY1-1)/n)
        bb[6] = math.floor(16*(MBBX2-1)/m)
        bb[7] = math.floor(16*(MBBY2-1)/n)

        for i in range(8):
            bits[ 8 + 4*i] = (bb[i] %  2)    
            bits[ 9 + 4*i] = (bb[i] %  4)>1.5
            bits[10 + 4*i] = (bb[i] %  8)>3.5
            bits[11 + 4*i] = (bb[i] % 16)>7.5

        # confidence map is custom
        CY = min(100, CY*100)
        CY = max(  0, CY*100)
        CM = min(100, CM*100) 
        CM = max(  0, CM*100)
        c_map = np.array([0,10,20,25,30,35,40,45,50,53,56,59,62,65,68,71,74,76,78,80,82,84,86,88,90,92,94,95,96,97,98,99])
        
        count = 0
        for c in c_map:
            if CY > c:
                bits[40] = (count %  2)
                bits[41] = (count %  4)> 1.5
                bits[42] = (count %  8)> 3.5
                bits[43] = (count % 16)> 7.5
                bits[44] = (count % 32)>15.5
            if CM > c:
                bits[45] = (count %  2)
                bits[46] = (count %  4)> 1.5
                bits[47] = (count %  8)> 3.5
                bits[48] = (count % 16)> 7.5
                bits[49] = (count % 32)>15.5
            count += 1
    
        #PIC_G = np.reshape(PIC_G, (m*n,1))
        PIC_G = np.sort(PIC_G, axis=None) #axis "none" should remove need to reshape
        Q30 = math.floor(PIC_G[round(0.3 * m*n)]/16)
        Q50 = math.floor(PIC_G[round(0.5 * m*n)]/16)
        Q70 = math.floor(PIC_G[round(0.7 * m*n)]/16)
        Q90 = math.floor(PIC_G[round(0.9 * m*n)]/16)

        bits[50] = (Q30 %  2)
        bits[51] = (Q30 %  4)>1.5
        bits[52] = (Q30 %  8)>3.5
        bits[53] = (Q30 % 16)>7.5
        bits[54] = (Q50 %  2)
        bits[55] = (Q50 %  4)>1.5
        bits[56] = (Q50 %  8)>3.5
        bits[57] = (Q50 % 16)>7.5
        bits[58] = (Q70 %  2)
        bits[59] = (Q70 %  4)>1.5
        bits[60] = (Q70 %  8)>3.5
        bits[61] = (Q70 % 16)>7.5
        bits[62] = (Q90 %  2)
        bits[63] = (Q90 %  4)>1.5
        bits[64] = (Q90 %  8)>3.5
        bits[65] = (Q90 % 16)>7.
        
        for i in range(3):
            PV = math.floor(PIX[i]/4)
            bits[66+6*i] = (PV %  2)        
            bits[67+6*i] = (PV %  4)> 1.5
            bits[68+6*i] = (PV %  8)> 3.5
            bits[69+6*i] = (PV % 16)> 7.5
            bits[70+6*i] = (PV % 32)>15.5
            bits[71+6*i] = (PV % 64)>31.5
        
        TEMP = max(0, TEMP)
        TEMP = min(63, TEMP)
        t = math.floor(16*TEMP/64)
        bits[84] = (t %  2)
        bits[85] = (t %  4)> 1.5
        bits[86] = (t %  8)> 3.5
        bits[87] = (t % 16)> 7.5

    else:  #HDD case
        bits[1] = (XN %  2)     
        bits[2] = (XN %  4)> 1.5
        bits[3] = (XN %  8)> 3.5
        bits[4] = (XN % 16)> 7.5
        bits[5] = (XN % 32)>15.5

        if WXA >= 25:
            bits[ 6:14] = np.ones((8,), dtype=int)
        elif WXA <= -25:
            bits[ 6:14] = np.zeros((8,), dtype=int)
        else:
            w = math.floor(256*(WXA+25)/50)
            bits[ 6] = (w %   2)
            bits[ 7] = (w %   4)>  1.5
            bits[ 8] = (w %   8)>  3.5
            bits[ 9] = (w %  16)>  7.5
            bits[10] = (w %  32)> 15.5
            bits[11] = (w %  64)> 31.5
            bits[12] = (w % 128)> 63.5
            bits[13] = (w % 256)>127.5
        
        if WXB >= 25:
            bits[14:22] = np.ones((8,), dtype=int)
        elif WXA <= -25:
            bits[14:22] = np.zeros((8,), dtype=int)
        else:
            w = math.floor(256*(WXB+25)/50)
            bits[14] = (w %   2)
            bits[15] = (w %   4)>  1.5
            bits[16] = (w %   8)>  3.5
            bits[17] = (w %  16)>  7.5
            bits[18] = (w %  32)> 15.5
            bits[19] = (w %  64)> 31.5
            bits[20] = (w % 128)> 63.5
            bits[21] = (w % 256)>127.5

        if WXC >= 25:
            bits[22:30] = np.ones((8,), dtype=int)
        elif WXC <= -25:
            bits[22:30] = np.zeros((8,), dtype=int)
        else:
            w = math.floor(256*(WXC+25)/50)
            bits[22] = (w %   2)
            bits[23] = (w %   4)>  1.5
            bits[24] = (w %   8)>  3.5
            bits[25] = (w %  16)>  7.5
            bits[26] = (w %  32)> 15.5
            bits[27] = (w %  64)> 31.5
            bits[28] = (w % 128)> 63.5
            bits[29] = (w % 256)>127.5

        if WYA >= 25:
            bits[30:39] = np.ones((9,), dtype=int)
             
        elif WYA <= -25:
            bits[30:39] = np.zeros((9,), dtype=int)
        else:
            w = math.floor(512*(WYA+25)/50)
            bits[30] = (w %   2)
            bits[31] = (w %   4)>  1.5
            bits[32] = (w %   8)>  3.5
            bits[33] = (w %  16)>  7.5
            bits[34] = (w %  32)> 15.5
            bits[35] = (w %  64)> 31.5
            bits[36] = (w % 128)> 63.5
            bits[37] = (w % 256)>127.5
            bits[38] = (w % 512)>255.5

        if WYB >= 25:
            bits[39:48] = np.ones((9,), dtype=int)
        elif WYB <= -25:
            bits[39:48] = np.zeros((9,), dtype=int)
        else:
            w = math.floor(512*(WYB+25)/50)
            bits[39] = (w %   2)
            bits[40] = (w %   4)>  1.5
            bits[41] = (w %   8)>  3.5
            bits[42] = (w %  16)>  7.5
            bits[43] = (w %  32)> 15.5
            bits[44] = (w %  64)> 31.5
            bits[45] = (w % 128)> 63.5
            bits[46] = (w % 256)>127.5
            bits[47] = (w % 512)>255.5

        if WYC >= 25:
            bits[48:57] = np.ones((9,), dtype=int)
        elif WYC <= -25:
            bits[48:57] = np.zeros((9,), dtype=int)
        else:
            w = math.floor(512*(WYC+25)/50)
            bits[48] = (w %   2)
            bits[49] = (w %   4)>  1.5
            bits[50] = (w %   8)>  3.5
            bits[51] = (w %  16)>  7.5
            bits[52] = (w %  32)> 15.5
            bits[53] = (w %  64)> 31.5
            bits[54] = (w % 128)> 63.5
            bits[55] = (w % 256)>127.5
            bits[56] = (w % 512)>255.5

        if WZA >= 25:
            bits[57:65] = np.ones((8,), dtype=int)
        elif WZA <= -25:
            bits[57:65] = np.zeros((8,), dtype=int)
        else:
            w = math.floor(256*(WZA+25)/50)
            bits[57] = (w %   2)
            bits[58] = (w %   4)>  1.5
            bits[59] = (w %   8)>  3.5
            bits[60] = (w %  16)>  7.5
            bits[61] = (w %  32)> 15.5
            bits[62] = (w %  64)> 31.5
            bits[63] = (w % 128)> 63.5
            bits[64] = (w % 256)>127.5
        
        if WZB >= 25:
            bits[65:73] = np.ones((8,), dtype=int)
        elif WZA <= -25:
            bits[65:73] = np.zeros((8,), dtype=int)
        else:
            w = math.floor(256*(WZB+25)/50)
            bits[65] = (w %   2)
            bits[66] = (w %   4)>  1.5
            bits[67] = (w %   8)>  3.5
            bits[68] = (w %  16)>  7.5
            bits[69] = (w %  32)> 15.5
            bits[70] = (w %  64)> 31.5
            bits[71] = (w % 128)> 63.5
            bits[72] = (w % 256)>127.5

        if WZC >= 25:
            bits[73:81] = np.ones((8,), dtype=int)
        elif WZC <= -25:
            bits[73:81] = np.zeros((8,), dtype=int)
        else:
            w = math.floor(256*(WZC+25)/50)
            bits[73] = (w %   2)
            bits[74] = (w %   4)>  1.5
            bits[75] = (w %   8)>  3.5
            bits[76] = (w %  16)>  7.5
            bits[77] = (w %  32)> 15.5
            bits[78] = (w %  64)> 31.5
            bits[79] = (w % 128)> 63.5
            bits[80] = (w % 256)>127.5

        CD = min(2999, CD) 
        CD = max(0, CD)
        CD = math.floor(32*CD/3000)
        bits[81] = (CD %  2)
        bits[82] = (CD %  4)>  1.5
        bits[83] = (CD %  8)>  3.5
        bits[84] = (CD % 16)>  7.5
        bits[85] = (CD % 32)> 15.5

        TEMP = min(63, TEMP)
        TEMP = max( 0, TEMP)
        t = math.floor(4*TEMP/64)
        bits[86] = (t % 2)
        bits[87] = (t % 4)> 1.5

    cs = np.sum(bits)
    bits[88] = (cs % 2)
    bits[89] = (cs % 4)>1.5

    return bits



def read_data_string(bits):
    """
    Given: bit string of the data (LSB first)
    Return: data (np array of numbers)
    See definition of numbers in write_data_string.py
    sorry I'm so rusty in python
    """
    m = 1080
    n = 1920

    TYPE = bits[0] #HDD (0) or HIO (1)

    if TYPE: #HIO
        PN = bits[1] + 2*bits[2] + 4*bits[3] + 8*bits[4] + 16*bits[5]
        YD = bits[6] #YOLO detected handrail?
        MD = bits[7] #MASK detected handrail?

        #YOLO bounding box
        YBBX1 = (m/16) * (0.5 + bits[ 8] + 2*bits[ 9] + 4*bits[10] + 8*bits[11])
        YBBY1 = (n/16) * (0.5 + bits[12] + 2*bits[13] + 4*bits[14] + 8*bits[15])
        YBBX2 = (m/16) * (0.5 + bits[16] + 2*bits[17] + 4*bits[18] + 8*bits[19])
        YBBY2 = (n/16) * (0.5 + bits[20] + 2*bits[21] + 4*bits[22] + 8*bits[23])
        #MASK bounding box
        MBBX1 = (m/16) * (0.5 + bits[24] + 2*bits[25] + 4*bits[26] + 8*bits[27])
        MBBY1 = (n/16) * (0.5 + bits[28] + 2*bits[29] + 4*bits[30] + 8*bits[31])
        MBBX2 = (m/16) * (0.5 + bits[32] + 2*bits[33] + 4*bits[34] + 8*bits[35])
        MBBY2 = (n/16) * (0.5 + bits[36] + 2*bits[37] + 4*bits[38] + 8*bits[39])

        c_map  = np.array([ 0,10,20,25,30,35,40,45,50,53,56,59,62,65,68,71,74,76,78,80,82,84,86,88,90,92,94,95,96,97,98, 99])
        c_map2 = np.array([10,20,25,30,35,40,45,50,53,56,59,62,65,68,71,74,76,78,80,82,84,86,88,90,92,94,95,96,97,98,99,100])
        c_map = 0.5*(c_map + c_map2)
        temp = bits[40] + 2*bits[41] + 4*bits[42] + 8*bits[43] + 16*bits[44]
        CY = c_map[temp]
        temp = bits[45] + 2*bits[46] + 4*bits[47] + 8*bits[48] + 16*bits[49]
        CM = c_map[temp]

        #grayscale values
        Q30 = (256/16) * (0.5 + bits[50] + 2*bits[51] + 4*bits[52] + 8*bits[53])
        Q50 = (256/16) * (0.5 + bits[54] + 2*bits[55] + 4*bits[56] + 8*bits[57])
        Q70 = (256/16) * (0.5 + bits[58] + 2*bits[59] + 4*bits[60] + 8*bits[61])
        Q90 = (256/16) * (0.5 + bits[62] + 2*bits[63] + 4*bits[64] + 8*bits[65])

        #pixel value
        PIX = np.zeros((3,), dtype=int)
        PIX[0] = (256/64) * (0.5 + bits[66] + 2*bits[67] + 4*bits[68] + 8*bits[69] + 16*bits[70] + 32*bits[71])
        PIX[1] = (256/64) * (0.5 + bits[72] + 2*bits[73] + 4*bits[74] + 8*bits[75] + 16*bits[76] + 32*bits[77])
        PIX[2] = (256/64) * (0.5 + bits[78] + 2*bits[79] + 4*bits[80] + 8*bits[81] + 16*bits[82] + 32*bits[83])

        TEMP = (64 / 16) *(0.5 + bits[84] + 2*bits[85] + 4*bits[86] + 8*bits[87])

        data = np.zeros((23,), dtype=float)
        data[ 0] = TYPE
        data[ 1] = PN
        data[ 2] = YD
        data[ 3] = MD
        data[ 4] = YBBX1
        data[ 5] = YBBY1
        data[ 6] = YBBX2
        data[ 7] = YBBY2
        data[ 8] = MBBX1
        data[ 9] = MBBY1
        data[10] = MBBX2
        data[11] = MBBY2
        data[12] = CY
        data[13] = CM
        data[14] = Q30
        data[15] = Q50
        data[16] = Q70
        data[17] = Q90
        data[18] = PIX[0]
        data[19] = PIX[1]
        data[20] = PIX[2]
        data[21] = TEMP

    else: #HDD data
        XN = bits[1] + 2*bits[2] + 4*bits[3] + 8*bits[4] + 16*bits[5]

        WXA = -25+(50/256)*(0.5+bits[ 6]+2*bits[ 7]+4*bits[ 8]+8*bits[ 9]+16*bits[10]+32*bits[11]+64*bits[12]+128*bits[13])
        WXB = -25+(50/256)*(0.5+bits[14]+2*bits[15]+4*bits[16]+8*bits[17]+16*bits[18]+32*bits[19]+64*bits[20]+128*bits[21])
        WXC = -25+(50/256)*(0.5+bits[22]+2*bits[23]+4*bits[24]+8*bits[25]+16*bits[26]+32*bits[27]+64*bits[28]+128*bits[29])
        WYA = -25+(50/512)*(0.5+bits[30]+2*bits[31]+4*bits[32]+8*bits[33]+16*bits[34]+32*bits[35]+64*bits[36]+128*bits[37]+256*bits[38])
        WYB = -25+(50/512)*(0.5+bits[39]+2*bits[40]+4*bits[41]+8*bits[42]+16*bits[43]+32*bits[44]+64*bits[45]+128*bits[46]+256*bits[47])
        WYC = -25+(50/512)*(0.5+bits[48]+2*bits[49]+4*bits[50]+8*bits[51]+16*bits[52]+32*bits[53]+64*bits[54]+128*bits[55]+256*bits[56])
        WZA = -25+(50/256)*(0.5+bits[57]+2*bits[58]+4*bits[59]+8*bits[60]+16*bits[61]+32*bits[62]+64*bits[63]+128*bits[64])
        WZB = -25+(50/256)*(0.5+bits[65]+2*bits[66]+4*bits[67]+8*bits[68]+16*bits[69]+32*bits[70]+64*bits[71]+128*bits[72])
        WZC = -25+(50/256)*(0.5+bits[73]+2*bits[74]+4*bits[75]+8*bits[76]+16*bits[77]+32*bits[78]+64*bits[79]+128*bits[80])

        CD = (3000 / 32) * (0.5 + bits[81] + 2*bits[82] + 4*bits[83] + 8*bits[84] + 16*bits[85])
        TEMP = (64 /  4) * (0.5 + bits[86] + 2*bits[87])

        data = np.zeros((14,), dtype=float)
        data[ 0] = TYPE
        data[ 1] = XN
        data[ 2] = WXA
        data[ 3] = WXB
        data[ 4] = WXC
        data[ 5] = WYA
        data[ 6] = WYB
        data[ 7] = WYC
        data[ 8] = WZA
        data[ 9] = WZB
        data[10] = WZC
        data[11] = CD
        data[12] = TEMP

    #checksum - same for both 
    cs = np.sum(bits[0:87])
    cs1 = (cs % 2)
    cs2 = (cs % 4)>1.5
    CS = (cs1 == bits[88]) and (cs2 == bits[89])
    data[data.size-1] = CS
    return data



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