# 商业转载请联系作者获得授权，非商业转载请注明出处。
# For commercial use, please contact the author for authorization. For non-commercial use, please indicate the source.
# 协议(License)：署名-非商业性使用-相同方式共享 4.0 国际 (CC BY-NC-SA 4.0)
# 作者(Author)：s-ubt-b
# 链接(URL)：https://qwqpap.xyz/
# 来源(Source)：天鹅绒房间

import cv2
import numpy as np
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


cv2.imshow('img', img)
if cv2.waitKey(0) == 27:
    cv2.destroyAllWindows()