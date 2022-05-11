import numpy as np
import math
from PIL import Image

def write_data_string(TYPE,PN,YD,MD,YBBX1,YBBY1,YBBX2,YBBY2,MBBX1,MBBY1,MBBX2,MBBY2,CY,CM,PIC,XN,WXA,WXB,WXC,WYA,WYB,WYC,WZA,WZB,WZC,CD,TEMP):
    """
    Return: bit string of the data (LSB first)
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

    sorry I'm so rusty in python
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
        CY = min(100, CY) 
        CY = max(  0, CY)
        CM = min(100, CM) 
        CM = max(  0, CM)
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