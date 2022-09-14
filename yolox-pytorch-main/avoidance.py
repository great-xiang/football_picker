import mmap, time, cv2
from PIL import Image
from io import BytesIO
import numpy as np
from yolo import YOLO
from math import sin, cos, fabs, acos, sqrt

pi = 3.1415926
p0 = 100  # 人距离多远才开始计算斥力



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
    input_angle = input_angle * pi / 180
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
    elif (obs[1] != 0 and people_cnt == 0):
        ang = ang_cal(vx, vy, ang)
    else:
        # 有人有球
        fhx, fhy = force_cal(fx, fy)
        vx, vy, x0, y0 = sep_cal(vx, vy, ch, fhx, fhy)
        ang = ang_cal(vx, vy, ang)

    # 换回角度制
    ang = ang * 180 / pi
    return ang, spe


while (1):
    angle = 10
    speed = 1
    #obs=[0,1,50,40,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
    obs = [0, 1, 50, 40, 0, 0, 0, 0, 2, 1, 140, 58, 2, 1, 80, 108, 2, 1, 43, 97, 2, 1, 70, 110]
    angle, speed = bizhang(angle, speed, obs)
    angle, speed = ("%8.2f" % angle), ("%8.2f" % speed)
    print("angle:", angle, "speed:", speed)
    time.sleep(1)

