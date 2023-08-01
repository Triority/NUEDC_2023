import math
import time 
import serial
from location import Location as mt

from threading import Thread

left_num = 1.1
right_num = 1.1

left_location = 1
right_location = 2

start_flag = 1

left_port = serial.Serial(
                            port="/dev/left_roll",
                            baudrate=115200,
                            bytesize=serial.EIGHTBITS,
                            parity=serial.PARITY_NONE,
                            stopbits=serial.STOPBITS_ONE,
                            )

right_port = serial.Serial(
                            port="/dev/right_roll",
                            baudrate=115200,
                            bytesize=serial.EIGHTBITS,
                            parity=serial.PARITY_NONE,
                            stopbits=serial.STOPBITS_ONE,
                            )

def left_speed_control(speed):
    str0 = "T"+str(speed)+"\n"
    left_port.write(str0.encode("utf-8"))

def right_speed_control(speed):
    str1 = "T"+str(speed)+"\n"
    right_port.write(str1.encode("utf-8"))
    
def get_right():
    global right_num,right_port

    strr = ""
    while True:
        if right_port.inWaiting() > 0:
            data = (right_port.read()).decode("utf-8")
            if data == '\n':
                #print(strr)
                #print("ll:"+strr)
                try:

                    strr = float(strr[:-1])
                    right_num = strr
                except:
                    pass
                strr = ""
            else:
                strr = strr + data
        
            #self.queue_get.get()

def get_left():

    global left_num,left_port

    strl = ""
    while True:
        if left_port.inWaiting() > 0:
            data = (left_port.read()).decode("utf-8")
            if data == '\n':
                #print(strl)                
                try:
                        #print(1)
                    # print("strl:"+strl)
                    strl = float(strl[:-1])
                    left_num = strl
                    # print("puted"+strl)
                except:
                    #print("g")
                    pass

                    #print("g")
                strl = ""
            else:
                strl = strl + data



left_proc = Thread(target=get_left)
right_proc = Thread(target=get_right)

left_proc.start()
right_proc.start()
run = mt(0,0)
time.sleep(1)
while left_num == -1.0:
    continue
start_left = left_num
while right_num == -1.0:
    continue
start_right = right_num

while True:
    time.sleep(0.01)   
    

   # right_location = right_queue.get()
    while left_num == -1.0:
        continue
    left_location = left_num - start_left
    while right_num == -1.0:
        continue
    right_location = right_num - start_right
    #print(left_location)

    location =run.count_location(left_location,right_location)
    print(location)
 