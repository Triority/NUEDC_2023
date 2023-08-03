from class_driver.class_driver import motor_driver
import time
import RPi.GPIO as GPIO 

import cv2 
import numpy as np




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


def switch(w,h):
    sw = 346 - ((346-303)/290) * w 
    sh = 382 - ((382-325)/250) * h 
    return sw,sh


def any():
    cap = cv2.VideoCapture(-1)
    ret, img = cap.read()
    img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
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
    retval, img = cv2.threshold(img,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)

    img = cv2.dilate(img, np.uint8(np.ones((3, 3))), 5)

    img = cv2.bitwise_not(img)
    contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnt = contours[0]
    rect = cv2.minAreaRect(cnt)  # 这里得到的是旋转矩形
    box = cv2.boxPoints(rect)  # 得到端点
    box = np.int32(box)
    print(img.shape)
    print(box)
    w = box[0][0]
    h = box[0][1]
    loc_w,loc_h = switch(w,h)
    left_driver.raw = float(loc_w)
    left_driver.lll = float(loc_h)
    time.sleep(0.4)
    w = box[1][0]
    h = box[1][1]
    loc_w,loc_h = switch(w,h)
    left_driver.raw = float(loc_w)
    left_driver.lll = float(loc_h)
    time.sleep(0.4)
    w = box[2][0]
    h = box[2][1]
    loc_w,loc_h = switch(w,h*0.9)
    left_driver.raw = float(loc_w)
    left_driver.lll = float(loc_h)
    time.sleep(0.4)
    w = box[3][0]
    h = box[3][1]
    loc_w,loc_h = switch(w,h*0.9)
    left_driver.raw = float(loc_w)
    left_driver.lll = float(loc_h)
    time.sleep(0.4)
    print(w)
    print(h)



if __name__ == '__main__':
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
            any()
            


    print("error")
    left_driver.raw = 370
    left_driver.lll = 370
    time.sleep(0.1)
    cap.release()
    exit()