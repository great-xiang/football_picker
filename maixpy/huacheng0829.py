import image, utime, time, gc, sys, sensor, random
import KPU as kpu
from fpioa_manager import fm
from Maix import GPIO, I2S
from machine import Timer, UART
from board import board_info

# ————————————————————————————————————
# 设置摄像头参数
# ————————————————————————————————————
input_size = (224, 224)
sensor.reset()
sensor.set_pixformat(sensor.RGB565)  # 设置摄像机的像素格式
sensor.set_framesize(sensor.QVGA)
sensor.set_windowing(input_size)
sensor.set_hmirror(False)  # 打开水平镜像模式
sensor.set_vflip(False)  # 打开竖直反转模式
sensor.skip_frames(time=2000)
sensor.run(1)

# ————————————————————————————————————————-————————
# 语音命令转换成磁场角
# [1:北边,2:东北,3:东边,4:东南,5:南边,6:西南,7:西边,8:西北]
# ————————————————————————————————————————————————
dir2mag = [100, 75, 75, 83, 95, 110, 120, 120]
mag = 0
# ——————————————————————————————————————————————————
# 速度初始化，范围[-200,200],speed[0]zuohou，speed[1]zuoqian，speed[2]youhou,speed[3]youqian，8位数据储存0~255
# ————————————————————————————————————————————————————
speed = [0, 150, 0, 150]

# ————————————————————————————————————
# 目标检测参数设置
# ————————————————————————————————————

labels = ['people', 'football', 'goal']
people = 0
football = 1
goal = 2
anchors = [4.92, 3.31, 0.69, 2.53, 1.12, 0.97, 0.47, 0.53, 1.09, 3.36]
task = None
#task = kpu.load("/sd/model-10899.kmodel")
#kpu.init_yolo2(task, 0.5, 0.3, 5, anchors)
obs = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

# ——————————————————————————————————————
# 循迹参数设置
# 阈值，晴天下午三点跑道(59,86,-18,10,2,15)，绿地(0,98,-12,4.-3,23),下午四点阴影(120,230,120,225,130,230)
# ——————————————————————————————————————
THRESHOLD = (105, 230, 110, 240, 130, 255)
green = (0, 0, 0, 0, 0, 0)
ROI = [1, 155, 220, 67]

# ——————————————————————————————————————————————
# 避障参数设置
# ——————————————————————————————————————————————
pi = 3.1415926
# 人距离多远才开始计算斥力
p0 = 100

# ————————————————————————————————————————————
# 通信接口设置
# ————————————————————————————————————————————

# arduino uart，uart UART 号，波特率,UART 数据宽度，奇偶校验位，停止位,串口接收超时时间,串口接收缓冲
fm.register(0, fm.fpioa.UART1_TX, force=True)
fm.register(1, fm.fpioa.UART1_RX, force=True)
uart1 = UART(UART.UART1, 9600, 8, 1, 0, timeout=1000, read_buf_len=4096)
# 语音识别uart
fm.register(2, fm.fpioa.UART2_TX, force=True)
fm.register(3, fm.fpioa.UART2_RX, force=True)
uart2 = UART(UART.UART2, 9600, 8, 1, 0, timeout=1000, read_buf_len=4096)
# 测距模块
fm.register(8, fm.fpioa.GPIO0)  # trig触发信号
fm.register(9, fm.fpioa.GPIO1)  # echo回响信号
trig = GPIO(GPIO.GPIO0, GPIO.OUT)
echo = GPIO(GPIO.GPIO1, GPIO.IN)


# ——————————————————————————————————————————————————————————————————————————————————————————————
# 向Arduino发送速度数据,turn_info偏转量，范围为（-1,1），负为左转弯，正为右转弯，乘以100加到speed上
# ——————————————————————————————————————————————————————————————————————————————————————————————
def to_arduino(v0=150, det_v=100, turn_info=0):
    global speed
    if (turn_info < 0):
        speed[3] += int((-turn_info) * det_v)
    else:
        speed[1] += int((turn_info) * det_v)
    string = ''
    for i in range(4):
        if (speed[i] < 10):
            string += '00'
        elif (speed[i] < 100):
            string += '0'
        string += str(speed[i])
    string += "00"
    uart1.write(string)
    speed = [0, v0, 0, v0]


def stop():
    uart1.write("00000000000000")


def getball():
    uart1.write("00000000000010")


def putball():
    uart1.write("00000000000001")


# ————————————————————————
# 从Arduino读取磁场角度数据，保存至全局变量mag
# ————————————————————————
def from_arduino():
    global mag
    read_data = uart1.read()
    if (read_data != None):
        data = str(read_data)[2:-1]
        data=data.split(',')
        print('磁场度数:%3s'%data[0],'速度设定值:%3s'%data[1],'%3s'%data[2],'真实速度:%3s'%data[3],'%3s'%data[4],'电机占空比%3s'%data[5],'%3s'%data[6])

# ————————————————————————————
# 读取语音指令
# ————————————————————————————
def listen():
    while (1):
        data = uart2.read()
        if (data != None):
            data = str(data)[10:-6]
            direction = dir2mag[int(data) - 1]
            print("语音指令方向磁场角：", direction)
            return direction
        utime.sleep_ms(100)


# ————————————————————————————————————
# 拍摄图片
# ————————————————————————————————————
def photo():
    number = random.randint(0, 10000000)
    sensor.snapshot().save("/sd/" + str(number) + ".jpg")


# ————————————————————————————————
# 测距, 单位为厘米
# ————————————————————————————————
def distance():
    trig.value(1)
    utime.sleep_us(10)
    trig.value(0)
    while not echo.value():
        pass
    t1 = time.ticks_us()
    while echo.value():
        pass
    t2 = time.ticks_us()
    return int((t2 - t1) * 0.034 / 2)


# ————————————————————————————
# 循迹
# ————————————————————————————
def detect_line():
    img = sensor.snapshot().binary([THRESHOLD], invert=True)
    img1 = sensor.snapshot()  # 正常读图片，为了找绿色做处理
    green_blobs = img1.find_blobs([green], merge=True)  # 所有的绿色块
    green_pixels = 0
    if (len(green_blobs)):
        for green_blob in green_blobs:
            green_pixels += green_blob.pixels()
    line = img.get_regression([THRESHOLD], roi=ROI, robust=True)

    rho_err = 0
    if (line):
        rho_err = (abs(line.rho()) - img.width() / 2)
        # if line.theta()>90:
        #     theta_err = line.theta()-180
        # else:
        #     theta_err = line.theta()
        # img.draw_line(line.line(), color=127)
        print("rho_err：", rho_err)
        turn_info = rho_err / 112
        if (green_pixels > 1500):
            turn_info = 0
        return turn_info
    return 0


# —————————————————————————————————————————————————————————————————————————————————————————————————————
# 目标检测,obs=[[football,置信度,纵坐标，横坐标],[goal,置信度，纵坐标，横坐标]，[people,置信度，纵坐标，横坐标]，……]
# 示例：obs=[[1,0.8,43,91],[0,0.6,83,92],[2,0.7,38,29],[0,0.9,48,99]]
# ——————————————————————————————————————————————————————————————————————————————————————————————————————
def yolo():
    img = sensor.snapshot()
    objects = kpu.run_yolo2(task, img)
    obs = []
    if objects:
        for i in objects:
            obs.append([i.classid(), i.value(), i.y() - i.h(), i.x() - 112])
    return obs


# -----------------------------------------------------------
# 避障
# ----------------------------------------------------------
def bizhang(turn_info, speed, obs):
    # 将turn_info变成弧度，参数传出时再变成turn_info
    input_speed = speed
    input_angle = turn_info * 90 * pi / 180
    obs = obs
    # 角度,最后结果,速度，最后结果
    ang, spe = 0., speed
    # x方向速度,y方向速度,这个只是中间计算的结果，vy / vx才有意义
    vx, vy = 0., 0.
    # x,y方向斥力,斥力可以很复杂需要改进再说;,x,y方向牵引力;x,y方向合力
    fx, fy, Fx, Fy, fhx, fhy = 0., 0., 0., 0., 0., 0.
    # 是否向右, 其实用1，-1更好（）,是否向前
    x0, y0 = 0, 0
    # 引力系数,斥力系数,合力系数，系数越大影响越大
    cF, cf, ch = 0.05, 1, 2
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

    # 根据不同的情况确定角度和速度
    if (obs[1] == 0 and people_cnt == 0):
        # 传入的角度速度即为所求
        pass
    # elif (obs[1] != 0 and people_cnt == 0):
    #   ang = ang_cal(vx, vy, ang)
    else:
        # 有人有球
        fhx, fhy = force_cal(fx, fy)
        vx, vy, x0, y0 = sep_cal(vx, vy, ch, fhx, fhy)
        ang = ang_cal(vx, vy, ang)

    # 换回角度制
    # ang = ang * 180 / pi/90
    ang = ang * 2 / pi
    return ang, spe



while(1):
    arduino()
    utime.sleep_ms(100)
    uart1.write("00010000000000")






#————————————————————————————————————————
#等待用户下达开始命令
#————————————————————————————————————————
direction=listen()

#————————————————————————————————————————
#调整初始方向
#————————————————————————————————————————

send_arduino(turn_info=0)
while (1):
    maganet()
    utime.sleep_ms(100)
    dif = direction - mag
    print("磁场差值:",dif)
    if (dif < 2 and dif > -2):
        stop()
        break
    elif (dif > 2):
        send_arduino(turn_info=1)
    else:
        send_arduino(turn_info=-1)


#————————————————————————————————————————
#识别是否进入跑道
#————————————————————————————————————————




#——————————————————————————————————————————
#循迹、避障、目标检测
#——————————————————————————————————————————
while(1):
    turn_info = detect_line()
    control_speed(turn_info=rho_err)
    obs=yolo()
    angle, speed = bizhang(angle, speed, obs)
    send_arduino()
    utime.sleep_ms(220)

#————————————————————————————————————————————
#抓球
#————————————————————————————————————————————

while(1):
    dist=distance()
    print(dist)
    if(dist<=5):
        getball()
        break
#————————————————————————————————————————————
#寻找足球门框，目标检测
#————————————————————————————————————————————

#————————————————————————————————————————————
#走向足球门框，目标检测，避障
#————————————————————————————————————————————


#————————————————————————————————————————————
#放球
#————————————————————————————————————————————
putball()

print("结束任务")


# 标准控制速度方法
# while(1):
# maganet()
# utime.sleep_ms(100)
# send_arduino(1)
# 等待
# while(1):
#     maganet()
#     utime.sleep_ms(100)
#     stop()
#测试帧率
#t = time.ticks_ms()
#t = time.ticks_ms() - t
#print(t)
