# General Setup
import time
import board
i2c = board.I2C()

# ESC Setup
import os
os.system ("sudo pigpiod")
import pigpio

# Voltmeter
import adafruit_ina260

# IMU
import adafruit_icm20x
# IMU = adafruit_icm20x.ICM20948(i2c)

# Connect ESC
ESC = 18
pi = pigpio.pi(); 
pi.set_servo_pulsewidth(ESC, 0)

# ESC Input Range
neutral_ESC_inp = 1488
max_CCW_ESC_inp = 1132
max_CW_ESC_inp = 1832

# Define ramp parameters
SLEEP_TIME = 2.0
DELTA = 50

# CCW drive function
def HDD_ccw_drive(sleep_time=SLEEP_TIME, delta=DELTA):
    """
    
    inp = neutral_ESC_inp
    [w_x0, w_y0, w_z0] = IMU.gyro
    while inp > max_CCW_ESC_inp:
        print(inp)
        pi.set_servo_pulsewidth(ESC,inp)
        inp = inp - delta
        time.sleep(sleep_time)
    time.sleep(5)
    [w_xf, w_yf, w_zf] = IMU.gyro
    time.sleep(sleep_time)
    pi.set_servo_pulsewidth(ESC,neutral_ESC_inp)
    
    return [w_x0, w_y0, w_z0, w_xf, w_yf, w_zf]
    """
    
    print("HDD Spin CCW!")
    
    return [-1.0, -2.0, -3.0, -1.0, -2.0, -3.0]
    

# CW drive function
def HDD_cw_drive(sleep_time=SLEEP_TIME, delta=DELTA):
    """
    inp = neutral_ESC_inp
    [w_x0, w_y0, w_z0] = IMU.gyro
    while inp < max_CW_ESC_inp:
        print(inp)
        pi.set_servo_pulsewidth(ESC,inp)
        inp = inp + delta
        time.sleep(sleep_time)
    time.sleep(5)
    [w_xf, w_yf, w_zf] = IMU.gyro
    time.sleep(sleep_time)
    pi.set_servo_pulsewidth(ESC,neutral_ESC_inp)
    
    return [w_x0, w_y0, w_z0, w_xf, w_yf, w_zf]
    """
    print("HDD Spin CW!")
    return [1.0, 2.0, 3.0, 1.0, 2.0, 3.0]
    
# HDD Stop Function
def HDD_stop():
    print("HDD Stop!")
    
    pi.set_servo_pulsewidth(ESC,neutral_ESC_inp)
    i = 1
    while i < 11:
        pi.set_servo_pulsewidth(ESC,neutral_ESC_inp)
        time.sleep(0.5)
        i = i + 1


