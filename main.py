from class_driver.class_driver import motor_driver
import time
import RPi.GPIO as GPIO 

import cv2 
import numpy as np



def img_get(img):

    size_min = 0
    # 滤波二值化
    # gs_frame = cv2.GaussianBlur(img, (gs, gs), 1)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # erode_hsv = cv2.erode(hsv, None, iterations=erode)
    inRange_hsv = cv2.inRange(hsv, np.array([153, 98, 33]), np.array([179, 255, 255]))
    
    img = inRange_hsv 
    # 外接计算
    cnts = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    target_list = []
    pos = []
    if size_min < 1:
        size_min = 1
    for c in cnts:
        if cv2.contourArea(c) < size_min:
            continue
        else:
            target_list.append(c)
    #print(target_list)
    for cnt in target_list:
        x, y, w, h = cv2.boundingRect(cnt)
        # cv2.rectangle(img0, (x, y), (x + w, y + h), (0, 255, 0), 2)
    if len(target_list) == 0:
        x = 0
        y = 0
    return x,y

def round():
    left_driver.raw = float(346)
    left_driver.lll = float(382)
    time.sleep(0.4)
    left_driver.raw = float(303)
    left_driver.lll = float(382)
    time.sleep(0.4)
    left_driver.raw = float(303)
    left_driver.lll = float(325)
    time.sleep(0.4)
    left_driver.raw = float(346)
    left_driver.lll = float(325)
    time.sleep(0.4)

def center():
    # 回到中点
    left_driver.raw = float(323) # 右小 左大
    left_driver.lll = float(358) # 上大 下小


def mini_any(img):
    point_human_1 = (180, 30)
    point_human_2 = (470, 280)

    pts3_d1 = np.float32([[210, 30], [450, 30], [180, 280], [470, 280]])  # 原图点
    pts3_d2 = np.float32([[180, 30], [470, 30], [180, 280], [470, 280]])  # 随机得到的四个点
    m = cv2.getPerspectiveTransform(pts3_d1, pts3_d2)  # 矩阵计算

    img = cv2.warpPerspective(img, m, (640, 480))


    #point_human_1 = (0, 0)
    #point_human_2 = (565, 460)
    img = img[point_human_1[1]:point_human_2[1], point_human_1[0]:point_human_2[0]]
    #print(img)
    point_x,point_y = img_get(img)
    return point_x,point_y

def any(img):
    left_driver.raw = float(323) # 右小 左大
    left_driver.lll = float(358) # 上大 下小
    
    
    point_human_1 = (180, 30)
    point_human_2 = (470, 280)

    pts3_d1 = np.float32([[210, 30], [450, 30], [180, 280], [470, 280]])  # 原图点
    pts3_d2 = np.float32([[180, 30], [470, 30], [180, 280], [470, 280]])  # 随机得到的四个点
    m = cv2.getPerspectiveTransform(pts3_d1, pts3_d2)  # 矩阵计算

    img = cv2.warpPerspective(img, m, (640, 480))


    #point_human_1 = (0, 0)
    #point_human_2 = (565, 460)
    img = img[point_human_1[1]:point_human_2[1], point_human_1[0]:point_human_2[0]]
    #print(img)
    point_x,point_y = img_get(img)
    print(point_x,point_y)
    img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

    retval, img = cv2.threshold(img,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)

    img = cv2.dilate(img, np.uint8(np.ones((3, 3))), 5)

    img = cv2.bitwise_not(img)
    contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnt = contours[0]
    rect = cv2.minAreaRect(cnt)  # 这里得到的是旋转矩形
    box = cv2.boxPoints(rect)  # 得到端点
    box = np.int32(box)
    #print(img.shape)
    print(box)
    return box, point_x, point_y


def moive_point(mv_x,mv_y):
    left_driver.raw = left_driver.raw-float(mv_x) # 右小 左大
    left_driver.lll = left_driver.lll-float(mv_y)


if __name__ == '__main__':
    cap = cv2.VideoCapture(-1)
    
    left_driver = motor_driver("/dev/left_roll",115200)
    
    left_driver.start()
    
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)

    GPIO.setup(26,GPIO.IN,pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(27,GPIO.IN,pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(21,GPIO.OUT)
    buzze = GPIO.PWM(21,1440)
    buzze.start(0)









    while True:

        if GPIO.input(27):
            center()
            print("center")
            buzze.ChangeDutyCycle(50)
            time.sleep(1)
            buzze.ChangeDutyCycle(0)
        elif GPIO.input(26):
            round()
            print("round")
            buzze.ChangeDutyCycle(50)
            time.sleep(1)
            buzze.ChangeDutyCycle(0)
        else:
            kp = 0.01
            ret, img = cap.read()
            kk,px,py=any(img)
            rate_1 = (kk[1][1] - kk[0][1])/(kk[1][0]-kk[0][0])
            list_1 = []
            for i in range(kk[0][0],kk[1][0],1):
                j = i * rate_1
                point = [i,j]
                list_1.append(point)
            
            while not GPIO.input(26) or not GPIO.input(27):
                
                for ps in list_1:
                    ret, img = cap.read()
                    px,py = mini_any(img)
                    dt_x = (ps[0] - px)*kp
                    dt_y = (ps[0] - py)*kp
                    moive_point(dt_x,dt_y)
                    print("dx"+str(dt_x))
                    print("dy"+str(dt_y))
                    time.sleep(0.05)
                


    print("error")
    left_driver.raw = 370
    left_driver.lll = 370
    time.sleep(0.1)
    cap.release()
    exit()