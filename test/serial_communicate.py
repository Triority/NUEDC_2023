import math
from multiprocessing import  Process
import serial
serial_port = serial.Serial(
        port="/dev/left_roll",
    baudrate=115200,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
)
serial_port.write("T2\n".encode())
str = ""



while 2:
    try:
        #print(run.add_motor(1,2))   
        if serial_port.inWaiting() > 0:
            data = serial_port.read().decode()
            if data == '\n':
                print(str)
                str = ""
            else:
                str = str + data
    except:
        serial_port.write("T0\r\n".encode())
        exit()


