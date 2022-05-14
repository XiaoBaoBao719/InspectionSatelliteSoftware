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
#Power_Sensor = adafruit_ina260.INA260(i2c)  remove comment during hardware
# IMU
import adafruit_icm20x
#IMU = adafruit_icm20x.ICM20948(i2c) remove comment during hardware

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
sleep_time_f = 1.0
delta = 50
[w_x1, w_y1, w_z1] = [0.0, 0.0, 0.0]

# CCW drive function
def HDD_ccw_drive(sleep_time=sleep_time, delta=delta):
    """
    
    inp = neutral_ESC_inp
    [w_x0, w_y0, w_z0] = IMU.gyro
    time.sleep(sleep_time)
    while inp > max_CCW_ESC_inp:
        print(inp)
        pi.set_servo_pulsewidth(ESC,inp)
        if inp == 1488 - 3*delta:
            [w_x1, w_y1, w_z1] = IMU.gyro
        inp = inp - delta
        time.sleep(sleep_time)
    time.sleep(sleep_time_f)
    [w_xf, w_yf, w_zf] = IMU.gyro
    HDD_current_f = Power_Sensor.current/1000
    time.sleep(sleep_time)
    pi.set_servo_pulsewidth(ESC,neutral_ESC_inp)
    
    return [w_x0, w_y0, w_z0, w_x1, w_y1, w_z1, w_xf, w_yf, w_zf, HDD_current_f]
    """
    
    print("HDD Spin CCW!")
    return [-1.0, -2.0, -3.0, -1.0, -2.0, -3.0]
    
# CW drive function
def HDD_cw_drive(sleep_time=sleep_time, delta=delta):
    """
    inp = neutral_ESC_inp
    [w_x0, w_y0, w_z0] = IMU.gyro
    time.sleep(5)
    while inp < max_CW_ESC_inp:
        print(inp)
        pi.set_servo_pulsewidth(ESC,inp)
        if inp == 1488 + 3*delta:
            [w_x1, w_y1, w_z1] = IMU.gyro
        inp = inp + delta
        time.sleep(sleep_time)
    time.sleep(sleep_time_f)
    [w_xf, w_yf, w_zf] = IMU.gyro
    HDD_current_f = Power_Sensor.current/1000
    time.sleep(sleep_time)
    pi.set_servo_pulsewidth(ESC,neutral_ESC_inp)
    return [w_x0, w_y0, w_z0, w_x1, w_y1, w_z1, w_xf, w_yf, w_zf, HDD_current_f]
    """
    print("HDD Spin CW!")
    return [1.0, 2.0, 3.0, 1.0, 2.0, 3.0, 2.0, 4.0, 6.0, 0.05]
    
def HDD_get_state():
    # [w_x, w_y, w_z] = IMU.gyro
    # result = IMU.gyro
    # return result

    print("HDD state: ", result)
    return [1.0, 2.0, 3.0]

# HDD Stop Function
def HDD_stop():
    print("HDD Stop!")
    
    pi.set_servo_pulsewidth(ESC,neutral_ESC_inp)
    i = 1
    while i < 11:
        pi.set_servo_pulsewidth(ESC,neutral_ESC_inp)
        time.sleep(0.5)
        i = i + 1

def HDD_print(results):
    [w_x0, w_y0, w_z0, w_x1, w_y1, w_z1, w_xf, w_yf, w_zf, HDD_current_f] = results
    print("Initial Gyro X:%.2f, Y: %.2f, Z: %.2f rads/s" % (w_x0, w_y0, w_z0))
    print("Middle Gyro X:%.2f, Y: %.2f, Z: %.2f rads/s" % (w_x1, w_y1, w_z1))
    print("Final Gyro X:%.2f, Y: %.2f, Z: %.2f rads/s" % (w_xf, w_yf, w_zf))
    print("Nominal Current Draw: %.2f" % (HDD_current_f))
    
    
