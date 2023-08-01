import serial
import threading
import socket

motor_angle = 0
port = 25502
serial_port = serial.Serial(
    port="/dev/left_roll",
    baudrate=115200,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
)


class ServerThread:
    global motor_angle, serial_port
    def __init__(self, addr, port):
        self.addr = addr
        self.port = port
    def start(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.bind((self.addr, self.port))
        s.listen(1)
        try:
            conn, addr = s.accept()
            p = threading.Thread(target=self.send, args=(conn,))
            q = threading.Thread(target=self.receive, args=(conn,))
            p.start()
            q.start()
        except:
            s.close()
            serial_port.write("T0\r\n".encode())
            exit()

    def send(self, conn):
        while True:
            print(motor_angle)
            conn.send(str(motor_angle).encode('utf-8'))
    
    def receive(self, conn):
        while True:
            data = conn.recv(1024).decode('utf-8')
            if data:
                serial_port.write((data+"\n").encode())


if __name__ == '__main__':
    Server_Thread = ServerThread('127.0.0.1', port)
    Server_Thread.start()
    angle_str = ""
    while 114514:
        try:
            if serial_port.inWaiting() > 0:
                data = serial_port.read().decode()
                if data == '\n':
                    motor_angle = angle_str
                    angle_str = ""
                else:
                    angle_str = angle_str + data
        except:
            serial_port.write("T0\r\n".encode())
            exit()