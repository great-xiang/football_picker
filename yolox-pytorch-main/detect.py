import cv2
import numpy as np

img = cv2.imread('img1.jpg', cv2.IMREAD_UNCHANGED)  # 无变换读取图片
gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)  # 灰度图
mask = cv2.inRange(gray, 255, 255)  # 二值化
kernel = np.ones((2, 2), np.uint8)  # 半径大小
erode = cv2.morphologyEx(mask, cv2.MORPH_ERODE, kernel)  # 腐蚀
n = [len(erode[:, 0]), len(erode[0, :])]
(x_point, y_point) = np.nonzero(erode)
if len(x_point) < 2:
    print(0)
f1 = np.polyfit(x_point, y_point, 1)
p1 = np.poly1d(f1)
point1 = (int(p1(0)), 0)
point2 = (int(p1(n[0])), n[0])
result = cv2.line(img, (int(n[1]/2), 0), (int(n[1]/2), n[0]), (0, 0, 255), 2)
result = cv2.line(result, point1, point2, (0, 255, 0), 2)
print(n[1]/2-p1(n[0]))  # 返回的是左右偏移的距离
cv2.imshow('inside', result)
cv2.waitKey(0)

