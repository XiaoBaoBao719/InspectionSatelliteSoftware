# General Setup
import time
# import board
# i2c = board.I2C()

# ESC Setup
import os
# os.system ("sudo pigpiod")
import pigpio

# Voltmeter
# import adafruit_ina260

# IMU
# import adafruit_icm20x
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
sleep_time = 2.0
delta = 50

# CCW drive function
def HDD_ccw_drive(sleep_time, delta):
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

# CW drive function
def HDD_cw_drive(sleep_time, delta):
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
    
# HDD Stop Function
def HDD_stop():
    pi.set_servo_pulsewidth(ESC,neutral_ESC_inp)
    i = 1
    while i < 11:
        pi.set_servo_pulsewidth(ESC,neutral_ESC_inp)
        time.sleep(0.5)
        i = i + 1


