import serial
import threading
import time
#from nav.location import Location as loc


class motor_driver:
    def __init__(self, port, baudrate):
        self.motor_angle = 0
        self.raw = 370
        self.lll = 370
        self.serial_port = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
        )
    
    def start(self):
        # p = threading.Thread(target=self.serial_read, args=())
        q = threading.Thread(target=self.serial_write, args=())
        # p.start()
        q.start()
        time.sleep(0.2)


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
            data = 'L' + str(self.raw) + '\n'
            self.serial_port.write(data.encode())
            data = "R" + str(self.lll) + '\n'
            self.serial_port.write(data.encode())
            time.sleep(0.05)
    
    def get_angle(self):
        return self.motor_angle
