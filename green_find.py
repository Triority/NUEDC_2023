import cv2
from simple_pid import PID
from class_driver.class_driver import motor_driver
import time
import numpy as np
from simple_pid import PID



def getpoints(img, hsvmin, hsv_max, gs, erode):
    img = cv2.GaussianBlur(img, (gs, gs), 1)
    img = img[120:600, 200:500]
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    hsv = cv2.erode(hsv, None, iterations=erode)
    dst = cv2.inRange(hsv, hsvmin, hsv_max)

    cnts = cv2.findContours(dst, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
    pos = []
    for cnt in cnts:
        x, y, w, h = cv2.boundingRect(cnt)
        cv2.rectangle(img, (x, y), (x + w, y + h), 255, 2)
        pos.append([int(x + w / 2), y + h / 2])
    return pos



if __name__ == '__main__':
    left_driver = motor_driver("/dev/left_roll",115200)
    right_driver = motor_driver("/dev/right_roll",115200)
    left_driver.start()
    right_driver.start()
    time.sleep(0.5)
    cap = cv2.VideoCapture(-1)
    pid_x = PID(0.0035, 0, 0, 0)
    pid_y = PID(0.0035, 0, 0, 0)
    try:
        while True:
            print('time:',time.time())
            for i in range(5):
                ret, frame = cap.read()
            # red
            red_points = getpoints(frame, np.array([153, 98, 33]), np.array([179, 255, 255]), 7, 0)
            # green
            green_points = getpoints(frame, np.array([46, 90, 179]), np.array([69, 206, 255]), 7, 0)

            
            if len(red_points)==0:
                print('NO red')
                red_points = red
                get_red = get_red + 1
            else:
                red = red_points.copy()
                get_red = 0
            if len(green_points)==0:
                print('NO green')
                continue
            else:
                print(red_points[0], green_points[0])

            error_x =  green_points[0][0] - red_points[0][0]
            error_y = green_points[0][1] - red_points[0][1]

            if (abs(error_x)^2 + abs(error_y)^2)<100 and get_red>0:
                move_y = 0
                move_x = 0
                left_driver.velocity = move_y
                right_driver.velocity = move_x
                
            else:
                move_y = -pid_y(error_y)
                move_x = pid_x(error_x)
            #move_y = 0.01*error_y
            #move_x = -0.01*error_x

            if move_x>0.3:
                move_x = 0.3
            elif move_x<-0.3:
                move_x = -0.3
            if move_y>0.3:
                move_y = 0.3
            elif move_y<-0.3:
                move_y = -0.3
            left_driver.velocity = move_y
            right_driver.velocity = move_x

            print(error_y, error_x, left_driver.target_position, right_driver.target_position, move_y, move_x)

    except KeyboardInterrupt:
        left_driver.velocity = 0
        right_driver.velocity = 0
        time.sleep(0.1)
        exit()