import time
import serial
               
ser = serial.Serial(            
    port='/dev/ttyAMA0',
    #baudrate = 9600,
    baudrate = 115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
)
        
buffer = "heyo"
dum_hdd_vals = str(420420420420420)

while 1:
    # x = ser.write(dum_hdd_vals.encode('utf-8'))
    x = ser.write(dum_hdd_vals.encode('utf-8'))
    #x = ser.read(100)
    print(x)