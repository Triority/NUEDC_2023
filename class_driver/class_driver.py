import serial
import threading
import time
from nav.location import Location as loc
class motor_driver:
    def __init__(self, port, baudrate):
        self.motor_angle = 0
        self.velocity = 0

        self.serial_port = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
        )
    
    def start(self):
        p = threading.Thread(target=self.serial_read, args=())
        q = threading.Thread(target=self.serial_write, args=())
        p.start()
        q.start()


    def serial_read(self):
        angle_str = ""
        while 114514:
            try:
                if self.serial_port.inWaiting() > 0:
                    data = self.serial_port.read().decode()
                    if data == '\n':
                        self.motor_angle = angle_str
                        angle_str = ""
                    else:
                        angle_str = angle_str + data
            except:
                self.velocity = 0
    
    def serial_write(self):
        while 114514:
            data = 'T' + str(self.velocity) + '\n'
            self.serial_port.write(data.encode())
            time.sleep(0.05)
    
    def get_angle(self):
        return self.motor_angle

if __name__ == '__main__':
    left_driver = motor_driver("/dev/left_roll",115200)
    right_driver = motor_driver("/dev/right_roll",115200)
    left_driver.start()
    right_driver.start()
    time.sleep(0.5)
    loccount = loc(left_driver.motor_angle,right_driver.motor_angle)
    try:
        location_list = []
        while True:
            left_driver.velocity = 0.5
            right_driver.velocity = 0.5
            location_list  = loccount.count_location(left_driver.motor_angle,right_driver.motor_angle)
            print(location_list)
    except:
        left_driver.velocity = 0
        time.sleep(0.1)
        exit()