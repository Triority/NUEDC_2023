# 商业转载请联系作者获得授权，非商业转载请注明出处。
# For commercial use, please contact the author for authorization. For non-commercial use, please indicate the source.
# 协议(License)：署名-非商业性使用-相同方式共享 4.0 国际 (CC BY-NC-SA 4.0)
# 作者(Author)：s-ubt-b
# 链接(URL)：https://qwqpap.xyz/
# 来源(Source)：天鹅绒房间

import cv2
import numpy as np
import time

# 'camera' or 'picture'
mode = 'camera'

if mode == 'camera':
    cap = cv2.VideoCapture(-1)


def update():
    global gs, erode, Hmin, Smin, Vmin, Hmax, Smax, Vmax, img, Hmin2, Hmax2, img0, size_min

    if mode == 'camera':
        ret, img0 = cap.read()
    img = img0.copy()

    gs = 7
    erode = 0
    Hmin = 107
    Smin = 0
    Vmin = 0
    Hmax = 179
    Smax = 255
    Vmax = 96
    size_min = 1300
    # 滤波二值化
    gs_frame = cv2.GaussianBlur(img, (gs, gs), 1)
    hsv = cv2.cvtColor(gs_frame, cv2.COLOR_BGR2HSV)
    erode_hsv = cv2.erode(hsv, None, iterations=erode)
    inRange_hsv = cv2.inRange(erode_hsv, np.array([Hmin, Smin, Vmin]), np.array([Hmax, Smax, Vmax]))
 
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
    for cnt in target_list:
        x, y, w, h = cv2.boundingRect(cnt)
        # cv2.rectangle(img0, (x, y), (x + w, y + h), (0, 255, 0), 2)
        pos.append([int(x),int(y),int(w),int(h)])
    print(pos)

if __name__ == "__main__":
    while 1:
        try:
            update()
        except:
            cap.release()