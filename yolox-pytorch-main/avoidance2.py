import mmap, time, cv2
from PIL import Image
from io import BytesIO
import numpy as np
from yolo import YOLO
from math import sin, cos, fabs, acos, sqrt

pi = 3.1415926
# 人距离多远才开始计算斥力
p0 = 100
# 初始化yolo目标检测模型
yolo = YOLO()

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
        time.sleep(0.01)
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


'''
输入obs数组，角度(inp_a)速度(inp_v)
    输出rob.ang和rob.spe
    1.读取数据，处理数据：
                将比较近的人读入 men.get();
                将球读入    ball.get();
                将门读入    door.get();
                将速度，角度读入 rob.get();
                                初始化rob .vx;rob.vy;rob.v0;rob.y0;
    2.分情况处理：
                没有人，没有球：按原inp_ang和inp_spe寻迹
                没有人，有球：向球运动
                都有：采用人工势场法
                     2.1 按照人的位置计算斥力fx，fy（实现见函数）
                     2.2 按照球的位置计算引力Fx，Fy
                     2.3 矢量合成Fhx = fx + Fx; Fhy = fy + Fy;
                     2.4 将输入速度分为vx，vy（
                     2.5 计算将要输出的速度:vx += ch * Fhx; vy += ch * Fhy; (c是常数)
                     2.6 vy / fabs(vx) < cs（c是常数）说明机器人正前方有人，要刹车，现在改成拐弯：
                                                                                           vx += c * Fxh / fabs(Fxh);
                                                                                           让vx大一点
     3.将vx和vy变成speed和angle

算法暂时还没有考虑门如何处理
速率始终保持最大，通过改变速度方向避障
'''


def bizhang(angle, speed, obs):
    # 输入值,三角函数按弧度制运算，参数传出时再变成角度值
    input_angle, input_speed = angle, speed
    input_angle = float(input_angle) * pi / 180
    obs = obs
    # 角度,最后结果,速度，最后结果
    ang, spe = 0.,speed
    # x方向速度,y方向速度,这个只是中间计算的结果，vy / vx才有意义
    vx, vy = 0., 0.
    # x,y方向斥力,斥力可以很复杂需要改进再说;,x,y方向牵引力;x,y方向合力
    fx, fy, Fx, Fy, fhx, fhy = 0., 0., 0., 0., 0., 0.
    # 是否向右, 其实用1，-1更好（）,是否向前
    x0, y0 = 0, 0
    # 引力系数,斥力系数,合力系数，系数越大影响越大
    cF, cf, ch = 1, 1, 1
    # 可允许的最大转角ca < pi（在需要刹车时会变成这个放向的转角，角的正负由新vx决定）
    ca = 3 * pi / 4
    if (input_angle > 0):
        x0 = 1
    else:
        x0 = 0
    if (fabs(input_angle) < pi / 2):
        y0 = 1
    else:
        y0 = 0
    # 计算速度分量
    vx = input_speed * sin(input_angle)
    vy = input_speed * cos(input_angle)
    # 计算人数和距离
    people_cnt = 0
    people_dis = [0, 0, 0, 0]
    for i in range(4):
        if (obs[9 + i * 4] == 0):
            break
        people_cnt += 1
        people_dis[i] = sqrt(obs[10 + i * 4] ** 2 + obs[11 + i * 4] ** 2)

    def force_cal(fx, fy):
        Fx = cF * obs[3]
        Fy = cF * obs[2]  # 引力

        # 每个人的斥力叠加
        for i in range(people_cnt):
            fx += -1 * cf * ((1 / people_dis[i]) - (1 / p0)) * pow(people_dis[i], -3) * obs[11 + i * 4]
            fy += -1 * cf * ((1 / people_dis[i]) - (1 / p0)) * pow(people_dis[i], -3) * obs[10 + i * 4]

        fhx = Fx + fx
        fhy = Fy + fy
        return fhx, fhy

    def sep_cal(vx, vy, ch, fhx, fhy):
        vx += ch * fhx
        vy += ch * fhy
        if (vx > 0):
            x0 = 1
        else:
            x0 = 0
        if (vy > 0):
            y0 = 1
        else:
            y0 = 0  # 记录新方向
        return vx, vy, x0, y0

    def ang_cal(vx, vy, ang):
        # 转角的绝对值的余弦,最大角的余弦,ca属于[0, pi],这个区间上余弦是连续的减函数
        cos_ang = vy / sqrt(vx ** 2 + vy ** 2)
        if (cos_ang > cos(ca)):
            # 转角不大
            ang = pow(-1, x0 + 1) * acos(cos_ang)
        else:
            ang = ca
        return ang

    #根据不同的情况确定角度和速度
    if (obs[1] == 0 and people_cnt == 0):
        # 传入的角度速度即为所求
        pass
    else:
        # 有人有球
        fhx, fhy = force_cal(fx, fy)
        vx, vy, x0, y0 = sep_cal(vx, vy, ch, fhx, fhy)
        ang = ang_cal(vx, vy, ang)

    # 换回角度制
    ang = ang * 180 / pi
    return ang, spe


while (1):
    img, Done = read_csharp()
    img_line = cv2.cvtColor(np.asarray(img), cv2.COLOR_RGB2BGR)
    angle=dectet_line(img_line)
    print("寻迹识别的角度：",angle)
    obs = yolo_detect(img)
    speed = 1.0
    angle, speed = bizhang(angle, speed, obs)
    angle, speed = ("%8.2f" % angle), ("%8.2f" % speed)
    print("输出角度：","angle:", angle, "speed:", speed)
    send_csharp(angle, speed, Done)
