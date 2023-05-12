import numpy as np
import math
from PIL import Image

def write_data_string(TYPE=0,PN=0,YD=0,MD=0,YBBX1=0,YBBY1=0,YBBX2=0,YBBY2=0,
                      MBBX1=0,MBBY1=0,MBBX2=0,MBBY2=0,CY=0,CM=0,PIC=np.array([[0]]),
                      XN=0,WXA=0,WXB=0,WXC=0,WYA=0,WYB=0,WYC=0,WZA=0,WZB=0,WZC=0,
                      TA=[0.0,0.0,0.0],TB=[0.0,0.0,0.0],TC=[0.0,0.0,0.0],CD=0,TEMP=0):
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
        CY = CY*100
        CM = CM*100
        CY = min(100, CY)
        CY = max(  0, CY)
        CM = min(100, CM) 
        CM = max(  0, CM)
        c_map = np.array([0,10,20,25,30,35,40,45,50,53,56,59,62,65,68,71,74,76,78,80,82,84,86,88,90,92,94,95,96,97,98,99])
        print("write Mask conf: ", CM, "\n write Yolo conf: ", CY)
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
        #print("CY bits: ",bits[44], " ", bits[43], " ", bits[42], " ", bits[41], " ",bits[40])  #delete
        #print("CM bits: ",bits[49], " ", bits[48], " ", bits[47], " ", bits[46], " ",bits[45]) #delete
        #PIC_G = np.reshape(PIC_G, (m*n,1))
        PIC_G = np.sort(PIC_G, axis=None) #axis "none" should remove need to reshape
        print("Q30: " ,PIC_G[round(0.3 * m*n)], " Q50: ",PIC_G[round(0.5 * m*n)], " Q70: ",PIC_G[round(0.7 * m*n)], " Q90: ", PIC_G[round(0.9 * m*n)])
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
        bits[65] = (Q90 % 16)>7.5
        
        print("R: ",PIX[0]," G: " ,PIX[1]," B: ",PIX[2])
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
        
        if XN % 2 == 0 :
            #insert HDD packet with mag data
            
            bits[6:14]  = IMU_to_bits(TA[0], 8, -80, 80)
            bits[14:22] = IMU_to_bits(TB[0], 8, -80, 80)    
            bits[22:30] = IMU_to_bits(TC[0], 8, -80, 80)
            
            bits[30:38] = IMU_to_bits(TA[1], 8, -80, 80)
            bits[38:46] = IMU_to_bits(TB[1], 8, -80, 80)
            bits[46:54] = IMU_to_bits(TC[1], 8, -80, 80)
            
            bits[54:62] = IMU_to_bits(TA[2], 8, -80, 80)
            bits[62:70] = IMU_to_bits(TB[2], 8, -80, 80)
            bits[70:78] = IMU_to_bits(TC[2], 8, -80, 80)
            #CD
            bits[78:86] = IMU_to_bits(CD, 8, 0, 2999)
                
        else:
            #WX
            bits[6:14]  = IMU_to_bits(WXA, 8, -25, 25)
            bits[14:22] = IMU_to_bits(WXB, 8, -25, 25)    
            bits[22:30] = IMU_to_bits(WXC, 8, -25, 25)
            #WY
            bits[30:38] = IMU_to_bits(WYA, 8, -25, 25)
            bits[38:46] = IMU_to_bits(WYB, 8, -25, 25)
            bits[46:54] = IMU_to_bits(WYC, 8, -25, 25)
            #WZ
            bits[54:63] = IMU_to_bits(WZA, 9, -25, 25)
            bits[63:72] = IMU_to_bits(WZB, 9, -25, 25)
            bits[72:81] = IMU_to_bits(WZC, 9, -25, 25)
            #CD
            bits[81:86] = IMU_to_bits(CD, 5, 0, 2999)

        TEMP = min(63, TEMP)
        TEMP = max( 0, TEMP)
        t = math.floor(4*TEMP/64)
        bits[86] = (t % 2)
        bits[87] = (t % 4)> 1.5

        cs = np.sum(bits)
        bits[88] = (cs % 2)
        bits[89] = (cs % 4)>1.5

    return bits

def IMU_to_bits (value, arr_size, min_val, max_val):
    #insert HDD packet with mag data
    if value >= max_val:
        print("Value too big")
        return np.ones((arr_size,), dtype=int)
    elif value <= min_val:
        print("Value too small")
        return np.zeros((arr_size,), dtype=int)
    else:
        value = math.floor( ((2 ** arr_size) * (value - min_val))/(max_val-min_val) )
        return np.flipud( np.array(list(np.binary_repr(value).zfill(arr_size))).astype(np.int8) )