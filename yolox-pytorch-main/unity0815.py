# -*- coding: <encoding name> -*- : # -*- coding: utf-8 -*-
import mmap, time, cv2, os, parl, paddle, math
from PIL import Image
from io import BytesIO
import numpy as np
from yolo import YOLO
import paddle.nn as nn
import paddle.nn.functional as F
from paddle.distribution import Normal

# 初始化yolo目标检测模型
yolo = YOLO()
# 强化学习策略网络输入维度，输出维度
obs_dim = 24
act_dim = 2
# 强化学习策略网络学习率,默认1e-3
LEARNING_RATE = 1e-2


# 读取c#传入数据
# mmap释放不了C#创建的共享内存；mmap读取共享内存会上锁，导致C#读取不了
def read_csharp():
    while (1):
        try:
            shmem2 = mmap.mmap(0, 10, "unitywrite", mmap.ACCESS_READ)
            unitywrite = shmem2.read(1).decode("ASCII")
            shmem2.close()
            # 即使unitywrite不存在，mmap也会读到数据，只不过为空，必须判断是否为数字，才可以跳出循环
            if (unitywrite.isdigit()):
                break
        except Exception as re:
            print(re)
            pass
        time.sleep(0.1)
    # 在读取unitywrite标志后再读取图片数据，是为了保证数据完整性。
    shmem1 = mmap.mmap(0, 100000, "img", mmap.ACCESS_READ)
    Done = int(shmem1.read(1).decode("ASCII"))
    img_size = int(shmem1.read(5).decode('ASCII'))
    img = Image.open(BytesIO(shmem1.read(img_size)))
    shmem1.close()
    return img, Done


# 向C#发送指令
# mmap创建的共享内存一旦调用close方法会释放内存，或者send_csharp结束调用也会释放内存，导致C#读取不了
def send_csharp(angle, speed, Done):
    angle = str(angle).encode('ASCII')
    speed = str(speed).encode('ASCII')
    Done = str(0).encode('ASCII')
    order = angle + speed + Done
    shmem3 = mmap.mmap(0, 20, 'order', mmap.ACCESS_WRITE)
    shmem3.write(order)
    # 创建一个C#读取标志，共享内存必须写入数据才会被创建
    shmem4 = mmap.mmap(0, 1, "pythonwrite", mmap.ACCESS_WRITE)
    shmem4.write(str("1").encode('ASCII'))
    time.sleep(0.1)
    # sleep结束后，共享内存会被释放
    return 0


def dectet_line(img):
    lower_white = np.array([0, 0, 225])
    upper_white = np.array([180, 30, 255])
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_white, upper_white)
    zero = np.zeros_like(mask)
    zero = cv2.fillPoly(zero, np.array([[[1, 154], [93, 121], [139, 121], [219, 174]]]), color=255)
    res = cv2.bitwise_and(mask, zero)
    white = cv2.countNonZero(res)
    m = cv2.moments(res, False)
    size = img.shape
    w = size[0]  # 宽度
    h = size[1]  # 高度
    try:
        Cx, Cy = m['m10'] / m['m00'], m['m01'] / m['m00']
    except ZeroDivisionError:
        Cx, Cy = h / 2, w / 2
    turn_info = (Cx - w / 2.0) / 10
    # 直角转弯
    # if (Cy > 67 * h / 100):
    #     turn_info = turn_info * 16.0 + 5
    #     return ("%8.2f" %(turn_info))

    if (white < 500 and abs(turn_info) < 0.5):
        turn_info = -0.5
    elif (turn_info > 2.0):
        turn_info = 0.5
    elif (turn_info < -3.5):
        turn_info = -3.5
    elif (turn_info > -1 and turn_info < 0.0):
        turn_info = -3.0
    return ("%8.2f" % (turn_info))


# 目标检测，返回物体识别结果和得分
def yolo_detect(img):
    # 返回结果物体种类，置信度，bottom，(left+right)/2,从左到右，从上到下
    my_result = yolo.detect_image(img, save_img=False)
    # 重新排列返回结果
    obs = np.zeros(24, dtype='float32')
    try:
        obs[0: 4] = my_result[my_result.index(0):my_result.index(0) + 4]
        del my_result[my_result.index(0):my_result.index(0) + 4]
    except:
        pass
    try:
        obs[4: 8] = my_result[my_result.index(2):my_result.index(2) + 4]
        del my_result[my_result.index(2):my_result.index(2) + 4]
    except:
        pass
    for i in range(len(my_result)):
        obs[8 + i] = my_result[0 + i]
    return obs


while (1):
    img, Done = read_csharp()
    obs = yolo_detect(img)
    python_Done = 0
    # 判断是否结束，Done==1抓到球了，Done==2碰到障碍物
    bottom = obs[2]
    abscissa = obs[3]
    if (obs[2] != 0):
        Angle = math.degrees(math.atan(abscissa / bottom))/10
    else:
        Angle = 0
    angle, speed = ("%8.2f" % Angle), ("%8.2f" % 1)
    print("图片的obs:", obs)
    print("angle:", angle, "speed:", speed)
    send_csharp(angle, speed, python_Done)
