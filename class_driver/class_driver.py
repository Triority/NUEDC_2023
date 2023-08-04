import serial
import threading
import time
from simple_pid import PID


class motor_driver:
    def __init__(self, port, baudrate):
        self.motor_angle = 0
        self.velocity = 0

        self.PID = False
        self.P = 10
        self.I = 0
        self.D = 1
        self.target_angle = 0
        self.output_limits = (-10, 10)
        self.error_k = 1
        self.PID_timesleep = 0.01

        self.error = 0

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
        time.sleep(0.2)

    def serial_read(self):
        angle_str = ""
        while 114514:
            try:
                if self.serial_port.inWaiting() > 0:
                    data = self.serial_port.read().decode()
                    if data == '\n':
                        self.motor_angle = float(angle_str)
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

    def angle_PID(self):
        pid = PID(self.P, self.I, self.D, setpoint=self.target_angle,output_limits = self.output_limits)
        while True:
            if self.PID:
                self.error  = self.error_k * (self.target_angle - self.motor_angle)
                self.velocity = pid(self.error)
                time.sleep(self.PID_timesleep)
            else:
                break

    def start_PID(self):
        r = threading.Thread(target=self.angle_PID, args=())
        r.start()

    def stop(self):
        self.PID = False
        self.velocity = 0
        time.sleep(0.1)



if __name__ == '__main__':
    left_driver = motor_driver("/dev/left_roll",115200)
    right_driver = motor_driver("/dev/right_roll",115200)
    left_driver.start()
    right_driver.start()
    time.sleep(1)
    left_driver.PID = True
    left_driver.start_PID()
    try:
        tar = left_driver.motor_angle

        while True:
            print(left_driver.velocity)
            left_driver.target_angle = tar-0.1

    except:
        left_driver.stop()
        right_driver.stop()
        print('stoped')
        left_driver.velocity = 0
        right_driver.velocity = 0
        time.sleep(0.1)
        exit()