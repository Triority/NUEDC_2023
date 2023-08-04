from class_driver.class_driver import motor_driver
import time
import RPi.GPIO as GPIO 

import cv2 
import numpy as np


def get_points(img):
    h = img.shape[0]
    w = img.shape[1]

    for i in range(h):  # 上
        if 255 in img[i]:
            white_index = np.where(img[i] == 255)
            point_a = [i, white_index[0][0]]
            break

    for i in range(w, 0, -1):  # 右
        if 255 in img[:, i - 1]:
            white_index = np.where(img[:, i - 1] == 255)
            point_b = [white_index[0][0], i]
            break

    for i in range(h, 0, -1):  # 下
        if 255 in img[i - 1]:
            white_index = np.where(img[i - 1] == 255)
            point_c = [i, white_index[0][0]]
            break

    for i in range(w):  # 左
        if 255 in img[:, i]:
            white_index = np.where(img[:, i] == 255)
            print('d:', )
            point_d = [white_index[0][0], i]
            break
    return [point_a, point_b, point_c, point_d]



last_x = 0
last_y = 0

def img_get(img):
    global last_x,last_y
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
        x = last_x
        y = last_y
    last_x = x
    last_y = y
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
    left_driver.lll = float(348) # 上大 下小


def mini_any(img):
    point_human_1 = (200, 30)
    point_human_2 = (460, 280)

    #point_human_1 = (0, 0)
    #point_human_2 = (565, 460)
    img = img[point_human_1[1]:point_human_2[1], point_human_1[0]:point_human_2[0]]
    #print(img)
    point_x,point_y = img_get(img)
    return point_y,point_x

def any(img):
    left_driver.raw = float(463) # 右小 左大
    left_driver.lll = float(339) # 上大 下小
    
    
    #point_human_1 = (180, 30)
    point_human_1 = (200, 30)
    point_human_2 = (460, 280)

    #point_human_1 = (0, 0)
    #point_human_2 = (565, 460)
    img = img[point_human_1[1]:point_human_2[1], point_human_1[0]:point_human_2[0]]
    #print(img)
    point_x,point_y = img_get(img)
    print(point_x,point_y)
    img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

    retval, img = cv2.threshold(img,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)

    #
    
    img = cv2.dilate(img, np.uint8(np.ones((3, 3))), 10)
    img = cv2.bitwise_not(img)
  ##img = cv2.erode(img, ,None, iterations=3)
    cv2.imwrite("233.png",img)
    box = get_points(img)
    #print(img.shape)
    print(box)
    return box, point_x, point_y


def moive_point(mv_x,mv_y):
    left_driver.raw = left_driver.raw-float(mv_x) # 右小 左大
    left_driver.lll = left_driver.lll-float(mv_y)



def list_cocu(x1,y1,x2,y2):
    step = 20
    fin_list = []
    x_now = x1 
    y_now = y1
    x_step = int(x2 - x1)/step
    y_step = int(y2 - y1)/step
    for i in range(0,step,1):
        x_now = x_now + x_step
        y_now = y_now + y_step
        post = [x_now,y_now]
        fin_list.append(post)
    return fin_list




def control(list_list,delay):
    kp = 0.02
    kd = 0.01
    ki = 0
    kpa = 0.01
    kda = 0.005
    kia = 0.005
    dt_y = 0
    dt_x = 0
    for ps in list_list:
        print(ps)
        for i in range(1,20):
            all_error_x = 0
            all_error_y = 0
            error_past_x = 0
            error_past_y = 0
            ret, img = cap.read()
            px,py = mini_any(img)
            error_now_y = (ps[0] - px)
            error_now_x = (ps[1] - py)
            dt_y = error_now_y*kp - (-error_past_y +error_now_y)*kd + all_error_y *ki
            dt_x = error_now_x*kpa - (-error_past_x +error_now_x)*kda + all_error_y *kia
            error_past_x = error_now_x
            error_past_y = error_now_y 
            all_error_x += error_now_x
            all_error_y += error_now_y
            print("px:"+str(px)+"py:"+str(py))
            print("tx:"+str(dt_x)+"ty:"+str(dt_y))
            moive_point(dt_x,dt_y)
            #print(ps[0] - px,ps[1] - py)
            #print(ps)
            #print(px,py)
            time.sleep(delay)





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
        print(1)
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
           
            ret, img = cap.read()
            kk,px,py=any(img)
            
            list_1 = list_cocu(kk[0][0],kk[0][1],kk[1][0],kk[1][1])
            
            list_2 = list_cocu(kk[1][0],kk[1][1],kk[2][0],kk[2][1])
            list_3 = list_cocu(kk[2][0],kk[2][1],kk[3][0],kk[3][1])
            list_4 = list_cocu(kk[3][0],kk[3][1],kk[0][0],kk[0][1])
            


            while not GPIO.input(26) or not GPIO.input(27):
                control(list_1,0.02)
                control(list_2,0.02)
                control(list_3,0.02)
                control(list_4,0.02)