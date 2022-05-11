import numpy as np
import math
from PIL import Image
import matplotlib.pyplot as plt
import cv2
from write_data_string import write_data_string
from read_data_string import read_data_string
from bits2char import bits2char
from char2bits import char2bits

num = 3802
path = r'C:\Users\admzu\Documents\MATLAB\HRVIP\HIO_FSW\TT1_iso_200_o1_ss_198.png'
PIC = cv2.imread(path)
m,n,d = PIC.shape

PN = range(num)
YD = 1 #YOLO detected handrail?
MD = 1 #MASK detected handrail?

bb_out = np.zeros((2,4,num))
w_out = np.zeros((3,3,num))
cy_out = np.zeros((num))
cm_out = np.zeros((num))
PN_out = np.zeros(num)
XN_out = np.zeros(num)
TEMP1_out = np.zeros(num)
TEMP2_out = np.zeros(num)
PIX_out = np.zeros(3)
CD_out = np.zeros(num)

YBBX1 = np.round(np.linspace(1,m,num))
YBBY1 = np.round(np.linspace(1,n,num))
YBBX2 = np.round(np.linspace(1,m,num))
YBBY2 = np.round(np.linspace(1,n,num))
MBBX1 = np.round(np.linspace(1,m,num))
MBBY1 = np.round(np.linspace(1,n,num))
MBBX2 = np.round(np.linspace(1,m,num))
MBBY2 = np.round(np.linspace(1,n,num))

CY = np.linspace(0,100,num)
CM = np.linspace(0,100,num)

TEMP = np.linspace(-5, 75, num)

#Test HIO string
#for i in range(num):
#    data_b  = write_data_string(1,i,YD,MD,YBBX1[i],YBBY1[i],YBBX2[i],YBBY2[i],MBBX1[i],MBBY1[i],MBBX2[i],MBBY2[i],CY[i],CM[i],PIC,0,0,0,0,0,0,0,0,0,0,0,TEMP[i])
#    data_c  = bits2char(data_b)
#    data_b2 = char2bits(data_c)
#    data_d  = read_data_string(data_b2)
#
#    PN_out[i]     = data_d[ 1]
#    bb_out[0,0,i] = data_d[ 4]
#    bb_out[0,1,i] = data_d[ 5]
#    bb_out[0,2,i] = data_d[ 6]
#    bb_out[0,3,i] = data_d[ 7]
#    bb_out[1,0,i] = data_d[ 8]
#    bb_out[1,1,i] = data_d[ 9]
#    bb_out[1,2,i] = data_d[10]
#    bb_out[1,3,i] = data_d[11]
#    cy_out[i]     = data_d[12]
#    cm_out[i]     = data_d[13]  
#    TEMP1_out[i]  = data_d[21]
#    PIX_out[0]    = data_d[18]
#    PIX_out[1]    = data_d[19]
#    PIX_out[2]    = data_d[20]

#plt.plot(PN, PN_out, '.')
#plt.plot((min(PN), max(PN)),(min(PN), max(PN)))
#plt.grid(True)
#plt.ylabel('OUTPUT')
#plt.xlabel('ACTUAL')
#plt.title('Photo Number')
#plt.show()

#plt.plot(YBBX1, bb_out[0,0,:], '.')
#plt.plot((min(YBBX1), max(YBBX1)),(min(YBBX1), max(YBBX1)))
#plt.grid(True)
#plt.ylabel('OUTPUT')
#plt.xlabel('ACTUAL')
#plt.title('Bounding Box YBBX1')
#plt.show()

#plt.plot(YBBY1, bb_out[0,1,:], '.')
#plt.plot((min(YBBY1), max(YBBY1)),(min(YBBY1), max(YBBY1)))
#plt.grid(True)
#plt.ylabel('OUTPUT')
#plt.xlabel('ACTUAL')
#plt.title('Bounding Box YBBY1')
#plt.show()

#plt.plot(YBBX2, bb_out[0,2,:], '.')
#plt.plot((min(YBBX2), max(YBBX2)),(min(YBBX2), max(YBBX2)))
#plt.grid(True)
#plt.ylabel('OUTPUT')
#plt.xlabel('ACTUAL')
#plt.title('Bounding Box YBBX2')
#plt.show()

#plt.plot(YBBY2, bb_out[0,3,:], '.')
#plt.plot((min(YBBY2), max(YBBY2)),(min(YBBY2), max(YBBY2)))
#plt.grid(True)
#plt.ylabel('OUTPUT')
#plt.xlabel('ACTUAL')
#plt.title('Bounding Box YBBY2')
#plt.show()

#plt.plot(MBBX1, bb_out[1,0,:], '.')
#plt.plot((min(MBBX1), max(MBBX1)),(min(MBBX1), max(MBBX1)))
#plt.grid(True)
#plt.ylabel('OUTPUT')
#plt.xlabel('ACTUAL')
#plt.title('Bounding Box MBBX1')
#plt.show()

#plt.plot(MBBY1, bb_out[1,1,:], '.')
#plt.plot((min(MBBY1), max(MBBY1)),(min(MBBY1), max(MBBY1)))
#plt.grid(True)
#plt.ylabel('OUTPUT')
#plt.xlabel('ACTUAL')
#plt.title('Bounding Box MBBY1')
#plt.show()

#plt.plot(MBBX2, bb_out[1,2,:], '.')
#plt.plot((min(MBBX2), max(MBBX2)),(min(MBBX2), max(MBBX2)))
#plt.grid(True)
#plt.ylabel('OUTPUT')
#plt.xlabel('ACTUAL')
#plt.title('Bounding Box MBBX2')
#plt.show()

#plt.plot(MBBY2, bb_out[1,3,:], '.')
#plt.plot((min(MBBY2), max(MBBY2)),(min(MBBY2), max(MBBY2)))
#plt.grid(True)
#plt.ylabel('OUTPUT')
#plt.xlabel('ACTUAL')
#plt.title('Bounding Box MBBY2')
#plt.show()

#plt.plot(CY, cy_out, '.')
#plt.plot((min(CY), max(CY)),(min(CY), max(CY)))
#plt.grid(True)
#plt.ylabel('OUTPUT')
#plt.xlabel('ACTUAL')
#plt.title('YOLO Confidence')
#plt.show()

#plt.plot(CM, cm_out, '.')
#plt.plot((min(CM), max(CM)),(min(CM), max(CM)))
#plt.grid(True)
#plt.ylabel('OUTPUT')
#plt.xlabel('ACTUAL')
#plt.title('Mask Confidence')
#plt.show()

#plt.plot(TEMP, TEMP1_out, '.')
#plt.plot((min(TEMP), max(TEMP)),(min(TEMP), max(TEMP)))
#plt.grid(True)
#plt.ylabel('OUTPUT')
#plt.xlabel('ACTUAL')
#plt.title('Temperature')
#plt.show()

#Test HDD string
W  = np.linspace(-30,30,num)
CD = np.linspace(0, 3500, num)
for i in range(num):
    data_b  = write_data_string(0,i,YD,MD,0,0,0,0,0,0,0,0,CY[i],CM[i],PIC,i,W[i],W[i],W[i],W[i],W[i],W[i],W[i],W[i],W[i],CD[i],TEMP[i])
    data_c  = bits2char(data_b)
    data_b2 = char2bits(data_c)
    data_d  = read_data_string(data_b2)

    XN_out[i]    = data_d[ 1]
    w_out[0,0,i] = data_d[ 2]
    w_out[0,1,i] = data_d[ 3]
    w_out[0,2,i] = data_d[ 4]
    w_out[1,0,i] = data_d[ 5]
    w_out[1,1,i] = data_d[ 6]
    w_out[1,2,i] = data_d[ 7]
    w_out[2,0,i] = data_d[ 8]
    w_out[2,1,i] = data_d[ 9]
    w_out[2,2,i] = data_d[10]
    CD_out[i]    = data_d[11]
    TEMP2_out[i] = data_d[12]

#plt.plot(PN, XN_out, '.')
#plt.plot((min(PN), max(PN)),(min(PN), max(PN)))
#plt.grid(True)
#plt.ylabel('OUTPUT')
#plt.xlabel('ACTUAL')
#plt.title('Exp Number')
#plt.show()

#plt.plot(TEMP, TEMP2_out, '.')
#plt.plot((min(TEMP), max(TEMP)),(min(TEMP), max(TEMP)))
#plt.grid(True)
#plt.ylabel('OUTPUT')
#plt.xlabel('ACTUAL')
#plt.title('Temperature2')
#plt.show()

#plt.plot(CD, CD_out, '.')
#plt.plot((min(CD), max(CD)),(min(CD), max(CD)))
#plt.grid(True)
#plt.ylabel('OUTPUT')
#plt.xlabel('ACTUAL')
#plt.title('Current Draw')
#plt.show()

plt.plot(W, w_out[0,0,:], '.')
plt.plot((min(W), max(W)),(min(W), max(W)))
plt.grid(True)
plt.ylabel('OUTPUT')
plt.xlabel('ACTUAL')
plt.title('WXA')
plt.show()

#plt.plot(W, w_out[0,1,:], '.')
#plt.plot((min(W), max(W)),(min(W), max(W)))
#plt.grid(True)
#plt.ylabel('OUTPUT')
#plt.xlabel('ACTUAL')
#plt.title('WXB')
#plt.show()

plt.plot(W, w_out[0,2,:], '.')
plt.plot((min(W), max(W)),(min(W), max(W)))
plt.grid(True)
plt.ylabel('OUTPUT')
plt.xlabel('ACTUAL')
plt.title('WXC')
plt.show()

plt.plot(W, w_out[1,0,:], '.')
plt.plot((min(W), max(W)),(min(W), max(W)))
plt.grid(True)
plt.ylabel('OUTPUT')
plt.xlabel('ACTUAL')
plt.title('WYA')
plt.show()

plt.plot(W, w_out[1,1,:], '.')
plt.plot((min(W), max(W)),(min(W), max(W)))
plt.grid(True)
plt.ylabel('OUTPUT')
plt.xlabel('ACTUAL')
plt.title('WYB')
plt.show()

plt.plot(W, w_out[1,2,:], '.')
plt.plot((min(W), max(W)),(min(W), max(W)))
plt.grid(True)
plt.ylabel('OUTPUT')
plt.xlabel('ACTUAL')
plt.title('WYC')
plt.show()

plt.plot(W, w_out[2,0,:], '.')
plt.plot((min(W), max(W)),(min(W), max(W)))
plt.grid(True)
plt.ylabel('OUTPUT')
plt.xlabel('ACTUAL')
plt.title('WZA')
plt.show()

plt.plot(W, w_out[2,1,:], '.')
plt.plot((min(W), max(W)),(min(W), max(W)))
plt.grid(True)
plt.ylabel('OUTPUT')
plt.xlabel('ACTUAL')
plt.title('WZB')
plt.show()

#plt.plot(W, w_out[2,2,:], '.')
#plt.plot((min(W), max(W)),(min(W), max(W)))
#plt.grid(True)
#plt.ylabel('OUTPUT')
#plt.xlabel('ACTUAL')
#plt.title('WZC')
#plt.show()