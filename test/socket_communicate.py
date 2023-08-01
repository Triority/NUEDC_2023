import threading
import socket
import time

port = 25502


class ServerThread:
    def __init__(self, addr, port):
        self.addr = addr
        self.port = port
    def start(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            s.connect((self.addr, self.port))
            p = threading.Thread(target=self.send, args=(s,))
            q = threading.Thread(target=self.receive, args=(s,))
            p.start()
            q.start()
            p.join()
            q.join()
        except:
            s.close()

    def send(self, s):
        while True:
            speed = 0
            for i in range(50):
                s.send(('T'+str(speed)).encode('utf-8'))
                speed  = speed + 0.3
                time.sleep(0.1)
            for i in range(50):
                s.send(('T'+str(speed)).encode('utf-8'))
                speed  = speed - 0.3
                time.sleep(0.1)


    
    def receive(self, s):
        while True:
            data = s.recv(1024).decode('utf-8')
            if data:
                print(data)


if __name__ == '__main__':
    Server_Thread = ServerThread('127.0.0.1', port)
    Server_Thread.start()