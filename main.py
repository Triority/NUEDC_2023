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
    point_human_1 = (200, 30)
    point_human_2 = (460, 280)

    pts3_d1 = np.float32([[210, 30], [450, 30], [180, 280], [470, 280]])  # 原图点
    pts3_d2 = np.float32([[180, 30], [470, 30], [180, 280], [470, 280]])  # 随机得到的四个点
    m = cv2.getPerspectiveTransform(pts3_d1, pts3_d2)  # 矩阵计算

    img = cv2.warpPerspective(img, m, (640, 480))


    #point_human_1 = (0, 0)
    #point_human_2 = (565, 460)
    img = img[point_human_1[1]:point_human_2[1], point_human_1[0]:point_human_2[0]]
    #print(img)
    point_x,point_y = img_get(img)
    return point_y,point_x

def any(img):
    left_driver.raw = float(323) # 右小 左大
    left_driver.lll = float(358) # 上大 下小
    
    
    #point_human_1 = (180, 30)
    point_human_1 = (200, 30)
    point_human_2 = (460, 280)

    pts3_d1 = np.float32([[210, 30], [450, 30], [180, 280], [470, 280]])  # 原图点
    pts3_d2 = np.float32([[180, 30], [470, 30], [180, 280], [470, 280]])  # 随机得到的四个点
    m = cv2.getPerspectiveTransform(pts3_d1, pts3_d2)  # 矩阵计算

    ##img = cv2.warpPerspective(img, m, (640, 480))


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
    left_driver.lll = left_driver.lll-float(mv_y)*0.5


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
            kp = 0.004
            ret, img = cap.read()
            kk,px,py=any(img)
            
            rate_1 = (kk[1][1] - kk[0][1])/(kk[1][0]-kk[0][0])
            list_1 = []
            step_y = 0.1 * rate_1
            j = kk[0][1]
            for i in range(10*kk[0][0],10*kk[1][0],1):
                i = i/10
                j = j + step_y
                point = [i,j]
                list_1.append(point)
            #print(list_1)    

            rate_2 = (kk[2][0]-kk[1][0])/(kk[2][1]-kk[1][1])
            list_2 = []
            step_x = 0.1* rate_2
            j = kk[1][0]
            for i in range(10*kk[1][1],10*kk[2][1],1):
                i = i/10
                j = j + step_x
                point = [j,i]
                list_2.append(point)
            #print(list_2)


            rate_3 = (kk[3][1] - kk[2][1])/(kk[3][0]-kk[2][0])
            list_3 = []
            step_y = 0.1 * rate_3
            j = kk[2][1]
            for i in range(10*kk[2][0],10*kk[3][0],-1):
                i = i/10
                j = j - step_y
                point = [i,j]
                list_3.append(point)
            #print(list_3)    


            rate_4 = (kk[0][0]-kk[3][0])/(kk[0][1]-kk[3][1])
            list_4 = []
            step_x = 0.1* rate_4
            print(step_x)
            j = kk[3][0]
            for i in range(10*kk[3][1],10*kk[0][1],-1):
                i = i/10
                j = j - step_x
                point = [j,i]
                list_4.append(point)
            # print(list_4)

            
            while not GPIO.input(26) or not GPIO.input(27):
                #print(list_1)
                #print(list_2)
                #print(list_3)
                #print(list_4)
                fr = 0.05
                """
                time.sleep(3)
                for ps in list_1:
                    ret, img = cap.read()
                    px,py = mini_any(img)
                    dt_x = (ps[0] - px)*kp
                    dt_y = (ps[1] - py)*kp
                    moive_point(dt_x,dt_y)
                    print(ps[0] - px,ps[1] - py)
                    
                    time.sleep(fr)
                print(1)
                time.sleep(0.2)
                for ps in list_2:
                    ret, img = cap.read()
                    px,py = mini_any(img)
                    dt_x = (ps[0] - px)*kp
                    dt_y = (ps[1] - py)*kp
                    moive_point(dt_x,dt_y,ps[1] - py)
                    print(ps[0] - px)
                    time.sleep(fr)
                print(2)
                time.sleep(0.2)
                for ps in list_3:
                    ret, img = cap.read()
                    px,py = mini_any(img)
                    dt_x = (ps[0] - px)*kp
                    dt_y = (ps[1] - py)*kp
                    moive_point(dt_x,dt_y)
                    print(ps[0] - px,ps[1] - py)   
                    time.sleep(fr)
                print(3)
                time.sleep(0.2)
                
                
                for ps in list_4:
                    ret, img = cap.read()
                    px,py = mini_any(img)
                    dt_x = (ps[0] - px)*kp
                    dt_y = (ps[1] - py)*kp
                    moive_point(dt_x,dt_y)
                    print(ps[0] - px,ps[1] - py)
                    time.sleep(fr)
                print(4)
                """
                time.sleep(0.2)
                for i in range(1,10000000):
                    ret, img = cap.read()
                    
                    py,px = mini_any(img)
                    print(px,py)
                    dt_x = (kk[2][1]-px)*kp
                    dt_y = (kk[2][0]-py)*kp
                    moive_point(dt_x,dt_y)
                    time.sleep(fr)
                time.sleep(0.2)
                for i in range(1,100):
                    ret, img = cap.read()
                    
                    px,py = mini_any(img)
                    dt_x = (kk[0][0]-px)*kp
                    dt_y = (kk[0][1]-py)*kp
                    moive_point(dt_x,dt_y)
                    time.sleep(fr)
                time.sleep(0.2)
                for i in range(1,100):
                    ret, img = cap.read()
                    

                    px,py = mini_any(img)
                    
                    dt_x = (kk[2][0]-px)*kp
                    dt_y = (kk[2][1]-py)*kp
                    moive_point(dt_x,dt_y)
                    time.sleep(fr)
                time.sleep(0.2)
                for i in range(1,100):
                    ret, img = cap.read()
                    
                    dt_x = (kk[3][0]-px)*kp
                    dt_y = (kk[3][1]-py)*kp
                    moive_point(dt_x,dt_y)
                    time.sleep(fr)
    print("error")
    left_driver.raw = 370
    left_driver.lll = 370
    time.sleep(0.1)
    cap.release()
    exit()