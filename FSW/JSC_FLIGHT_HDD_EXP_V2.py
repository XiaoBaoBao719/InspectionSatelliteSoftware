# General Setup
import time
import board
import math

# ESC Setup
import os
import pigpio

# Create I2C object
i2c = board.I2C()

# Voltmeter
import adafruit_ina260

try:
    Power_Sensor = adafruit_ina260.INA260(i2c)
except Exception:
    print("Bad I2C Power Sensor setup")

# IMU
import adafruit_icm20x
try:
    IMU = adafruit_icm20x.ICM20948(i2c)
except Exception:
    print("Bad I2C IMU Sensor setup")

PI = 3.1415926536

# Connect ESC
ESC = 18
pi = pigpio.pi(); 
pi.set_servo_pulsewidth(ESC, 0)

# ESC Input Range
neutral_ESC_inp = 1488
max_CCW_ESC_inp = 1132
max_CW_ESC_inp = 1832

#flight param
B_Gain_X = 1.0549
B_Gain_Y = 0.9888
B_Gain_Z = 0.9433
B_Offset_X = -27.9300
B_Offset_Y = 68.7450
B_Offset_Z =  14.8050

# Define ramp parameters
sleep_time = 2.0
sleep_time_f = 1.0
delta = 50
[w_x1, w_y1, w_z1, T_x1, T_y1, T_z1] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


# CCW drive function
def HDD_ccw_drive(sleep_time, delta):
    inp = neutral_ESC_inp
    [w_x0, w_y0, w_z0, T_0] = getIMU()
    time.sleep(sleep_time)
    while inp > max_CCW_ESC_inp:
        #print(inp)
        pi.set_servo_pulsewidth(ESC,inp)
        if inp == 1488 - 3*delta:
            [w_x1, w_y1, w_z1, T_1] = getIMU()
        inp = inp - delta
        time.sleep(sleep_time)
    time.sleep(sleep_time_f)
    [w_xf, w_yf, w_zf, T_f] = getIMU()
    HDD_current_f = getCurrent()
    time.sleep(sleep_time)
    pi.set_servo_pulsewidth(ESC,neutral_ESC_inp)
    
    return [w_x0, w_y0, w_z0, w_x1, w_y1, w_z1,
            w_xf, w_yf, w_zf, T_0, T_1, T_f, HDD_current_f]

# CW drive function
def HDD_cw_drive(sleep_time, delta):
    inp = neutral_ESC_inp
    [w_x0, w_y0, w_z0, T_0] = getIMU()
    time.sleep(sleep_time)
    while inp < max_CW_ESC_inp:
        #print(inp)
        pi.set_servo_pulsewidth(ESC,inp)
        if inp == 1488 + 3*delta:
            [w_x1, w_y1, w_z1, T_1] = getIMU()
        inp = inp + delta
        time.sleep(sleep_time)
    time.sleep(sleep_time_f)
    [w_xf, w_yf, w_zf, T_f] = getIMU()
    
    HDD_current_f = getCurrent()
    
    time.sleep(sleep_time)
    pi.set_servo_pulsewidth(ESC,neutral_ESC_inp)
    return [w_x0, w_y0, w_z0, w_x1, w_y1, w_z1,
            w_xf, w_yf, w_zf, T_0, T_1, T_f, HDD_current_f]
    
# HDD Stop Function
def HDD_stop():
    pi.set_servo_pulsewidth(ESC,neutral_ESC_inp)
    i = 1
    while i < 11:
        pi.set_servo_pulsewidth(ESC,neutral_ESC_inp)
        time.sleep(0.5)
        i = i + 1

def HDD_print(results):
    [w_x0, w_y0, w_z0, w_x1, w_y1, w_z1,
            w_xf, w_yf, w_zf, T_0, T_1, T_f, HDD_current_f] = results
    print("Initial Gyro X:%.2f, Y: %.2f, Z: %.2f degs/s" % (w_x0, w_y0, w_z0))
    print("Middle Gyro X:%.2f, Y: %.2f, Z: %.2f degs/s" % (w_x1, w_y1, w_z1))
    print("Final Gyro X:%.2f, Y: %.2f, Z: %.2f degs/s" % (w_xf, w_yf, w_zf))
    print("Initial Mag X:%.2f, Y: %.2f, Z: %.2f uT" % (T_0[0],T_0[1],T_0[2]))
    print("Middile Mag X:%.2f, Y: %.2f, Z: %.2f uT" % (T_1[0],T_1[1],T_1[2]))
    print("Final Mag X:%.2f, Y: %.2f, Z: %.2f uT" % (T_f[0],T_f[1],T_f[2]))
    print("Nominal Current Draw: %.2f mA" % (HDD_current_f))
    

def getIMU():
    try:
        #[w_x, w_y, w_z] = IMU.gyro
        
        #throw some filtering in there
        [T_x1, T_y1, T_z1] = IMU.magnetic
        [w_x1, w_y1, w_z1] = IMU.gyro
        time.sleep(0.008)
        [T_x2, T_y2, T_z2] = IMU.magnetic
        [w_x2, w_y2, w_z2] = IMU.gyro
        time.sleep(0.008)
        [T_x3, T_y3, T_z3] = IMU.magnetic
        [w_x3, w_y3, w_z3] = IMU.gyro
        time.sleep(0.008)
        [T_x4, T_y4, T_z4] = IMU.magnetic
        [w_x4, w_y4, w_z4] = IMU.gyro
        time.sleep(0.008)
        [T_x5, T_y5, T_z5] = IMU.magnetic
        [w_x5, w_y5, w_z5] = IMU.gyro
        #print("x1:"+str(w_x1*(180/PI)))
        #print("x2:"+str(w_x2*(180/PI)))
        #print("x3:"+str(w_x3*(180/PI)))
        #print("x4:"+str(w_x4*(180/PI)))        
        #print("x5:"+str(w_x5*(180/PI)))
        #print("y1:"+str(w_y1*(180/PI)))
        #print("y2:"+str(w_y2*(180/PI)))
        #print("y3:"+str(w_y3*(180/PI)))
        #print("y4:"+str(w_y4*(180/PI)))        
        #print("y5:"+str(w_y5*(180/PI)))
        #print("z1:"+str(w_z1*(180/PI)))
        #print("z2:"+str(w_z2*(180/PI)))
        #print("z3:"+str(w_z3*(180/PI)))
        #print("z4:"+str(w_z4*(180/PI)))        
        #print("z5:"+str(w_z5*(180/PI))) 
        w_x =0.20 * (w_x1 + w_x2 + w_x3 + w_x4 + w_x5);
        w_y =0.20 * (w_y1 + w_y2 + w_y3 + w_y4 + w_y5);
        w_z =0.20 * (w_z1 + w_z2 + w_z3 + w_z4 + w_z5);
        T_x =0.20 * (T_x1 + T_x2 + T_x3 + T_x4 + T_x5);
        T_y =-0.20 * (T_y1 + T_y2 + T_y3 + T_y4 + T_y5);
        T_z =-0.20 * (T_z1 + T_z2 + T_z3 + T_z4 + T_z5);
        T = [T_x,T_y,T_z];
        T = [(B_Gain_X * (T_x -  B_Offset_X)), (B_Gain_Y * (T_y -  B_Offset_Y)), (B_Gain_Z * (T_z -  B_Offset_Z))]
        return [w_x*(180/PI), w_y*(180/PI), w_z*(180/PI), T]
    except Exception:
        print("Error reading IMU")
        output = [0.0, 0.0, 0.0, [0.0, 0.0, 0.0]]
    return output

def getCurrent():
    try:
        return Power_Sensor.current
    except:
        print("Issue reading the Power sensor")
    return -1.0

#""" testing for IMU data and CW CWW rotation

#HDD_cw_drive(sleep_time, delta)
"""
with open("valid_output.txt", 'a') as f:
    
    for x in range(300):
        [w_x, w_y, w_z, T] = getIMU()
        f.write(" %.3f, %.3f, %.3f,"% (T[0], T[1], T[2]))
        time.sleep(0.2)
        """
        
"""
[w_x0, w_y0, w_z0, w_x1, w_y1, w_z1, w_xf, w_yf, w_zf, T_0, T_1, T_f, HDD_current_f] = HDD_ccw_drive(sleep_time, delta)
#[w_x0, w_y0, w_z0, w_x1, w_y1, w_z1, w_xf, w_yf, w_zf, HDD_current_f] = HDD_cw_drive(sleep_time, delta)
HDD_stop()
print("Initial Gyro X:%.2f, Y: %.2f, Z: %.2f degs/s" % (w_x0, w_y0, w_z0))
print("Middle Gyro X:%.2f, Y: %.2f, Z: %.2f degs/s" % (w_x1, w_y1, w_z1))
print("Final Gyro X:%.2f, Y: %.2f, Z: %.2f degs/s" % (w_xf, w_yf, w_zf))
print("Initial Mag X:%.2f, Y: %.2f, Z: %.2f uT" % (T_0[0],T_0[1],T_0[2]))
print("Middile Mag X:%.2f, Y: %.2f, Z: %.2f uT" % (T_1[0],T_1[1],T_1[2]))
print("Final Mag X:%.2f, Y: %.2f, Z: %.2f uT" % (T_f[0],T_f[1],T_f[2]))
print("Nominal Current Draw: %.2f \n\n" % (HDD_current_f))

[w_x0, w_y0, w_z0, w_x1, w_y1, w_z1, w_xf, w_yf, w_zf, T_0, T_1, T_f, HDD_current_f] = HDD_cw_drive(sleep_time, delta)
HDD_stop()
print("Initial Gyro X:%.2f, Y: %.2f, Z: %.2f degs/s" % (w_x0, w_y0, w_z0))
print("Middle Gyro X:%.2f, Y: %.2f, Z: %.2f degs/s" % (w_x1, w_y1, w_z1))
print("Final Gyro X:%.2f, Y: %.2f, Z: %.2f degs/s" % (w_xf, w_yf, w_zf))
print("Initial Mag X:%.2f, Y: %.2f, Z: %.2f uT" % (T_0[0],T_0[1],T_0[2]))
print("Middile Mag X:%.2f, Y: %.2f, Z: %.2f uT" % (T_1[0],T_1[1],T_1[2]))
print("Final Mag X:%.2f, Y: %.2f, Z: %.2f uT" % (T_f[0],T_f[1],T_f[2]))
print("Nominal Current Draw: %.2f" % (HDD_current_f))
"""