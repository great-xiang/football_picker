import image, utime, time, gc, sensor,Maix,sys,random
import KPU as kpu
from fpioa_manager import fm
from Maix import GPIO, I2S
from machine import Timer, UART

# ————————————————————————————————————————-———————————————————————————————————————————————————
# 全局变量
# ———————————————————————————————————————————————————————————————————————————————————————————
# 语音命令转换成磁场角[1:北边,2:东北,3:东边,4:东南,5:南边,6:西南,7:西边,8:西北]
dir2mag = [100, 75, 75, 83, 95, 110, 120, 120]
# 全局变量磁场角度
mag = 0
# 左右轮转速
rpm1, rpm0 = 0, 0
turn_info = 0
# 循迹参数设置
threshold=(100,255)
green=(10,45,-16,-7,12,22)
#ROI = (0,20,32,12)
ROI=[80, 90, 130, 120]
kernel_size = 1
kernel = [-1, -1, -1,
          -1, +8, -1,
          -1, -1, -1]
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

# ————————————————————————————————————————————————————————————————————————————————————————————————————
# 目标检测参数设置
# ——————————————————————————————————————————————————————————————————————————————————————————————————————

labels = ['people', 'goal', 'football']
people = 0
football = 2
goal = 1
anchors = [0.44, 0.47, 5.34, 3.62, 2.28, 1.94, 0.75, 1.06, 0.81, 2.97]
task = None
## 显示堆内存
#print(Maix.utils.heap_free() / 1024)
## 显示栈内存
#print(gc.mem_free() / 1024)
task = kpu.load("/sd/model-11393.kmodel")
kpu.init_yolo2(task, 0.5, 0.3, 5, anchors)
## 显示堆内存
#print(Maix.utils.heap_free() / 1024)
## 显示栈内存
#print(gc.mem_free() / 1024)
obs = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

# ——————————————————————————————————————————————————————————————————————————————————————————
# 通信接口设置
# arduino uart，uart UART 号，波特率,UART 数据宽度，奇偶校验位，停止位,串口接收超时时间,串口接收缓冲
# ——————————————————————————————————————————————————————————————————————————————————————————
fm.register(0, fm.fpioa.UART1_TX, force=True)
fm.register(1, fm.fpioa.UART1_RX, force=True)
uart1 = UART(UART.UART1, 115200, 8, None, 0, timeout=1000, read_buf_len=4096)
# 语音识别uart
fm.register(2, fm.fpioa.UART2_TX, force=True)
fm.register(3, fm.fpioa.UART2_RX, force=True)
uart2 = UART(UART.UART2, 9600, 8, 1, 0, timeout=1000, read_buf_len=4096)
# 测距模块
fm.register(8, fm.fpioa.GPIO0)  # trig触发信号
fm.register(9, fm.fpioa.GPIO1)  # echo回响信号
trig = GPIO(GPIO.GPIO0, GPIO.OUT)
echo = GPIO(GPIO.GPIO1, GPIO.IN)


# 定时器回调函数
def arduino(self):
    global mag, rpm0, rpm1
    read_data = uart1.read()
    if (read_data != None):
        data = str(read_data)[2:-1]
        data = data.split(',')
        try:
            data = data[data.index("a") + 1:data.index("b")]
            mag = int(data[0])
            rpm0, rpm1 = 4*int(data[1]), 4*int(data[2])
            print('磁场度数:%3s' % data[0], '真实速度:%4s' % rpm0, '%4s' % rpm1, 'pwm%3s' % data[3], '%3s' % data[4], '%3s' % data[5], '%3s' % data[6])
        except:
            pass


# 与arduino通信，100ms自动通信一次
tim = Timer(Timer.TIMER0, Timer.CHANNEL0, mode=Timer.MODE_PERIODIC, period=50, callback=arduino)
tim.start()


# ——————————————————————————————————————————————————————————————————————————————————————————————
# 向Arduino发送速度数据,turn_info偏转量，范围为（-1,1），负为左转弯，正为右转弯，乘以100加到speed上
# ————————————————————————————————————————————————————————————————————————————————————————
# PID控制类
# P主管响应，I减小静差，D抑制震荡。
# p比例，p很小，柔软；p很大，生硬。
# i积分，i很小，到不了目标值；i很大，超过目标值震荡。
# d微分，d很小，围绕目标值震荡；d很大，很难达到目标值。


class PID():
    def __init__(self, name):
        # PID参数设置
        self.kp, self.kd = 3, 0.6
        self.lastpwm=0
        self.name = name

    def run(self, Set_rpm, real_rpm):
        error = Set_rpm - real_rpm
        pwm = int(self.kp * error + self.kd*self.lastpwm)
        self.lastpwm = pwm
        if (pwm > 255):
            pwm = 255
        if (pwm < -255):
            pwm = -255
        #print('%s' % self.name, '速度设定值:%4s' % Set_rpm, '真实速度:%4s' % real_rpm, '电机占空比%4s' % pwm)
        return pwm

#class PID():
    #def __init__(self, name):
        ## PID参数设置
        #self.kp, self.ki, self.kd = 0.6, 0.1, 0.05
        #self.errSum, self.lastErr = 0, 0
        #self.name = name

    #def run(self, Set_rpm, real_rpm):
        #error = Set_rpm - real_rpm
        #self.errSum += error
        #dErr = error - self.lastErr
        #pwm = int(self.kp * error + self.ki * self.errSum + self.kd * dErr)
        #self.lastErr = error
        #if (pwm > 255):
            #pwm = 255
        #if (pwm < -255):
            #pwm = -255
        ##print('%s' % self.name, '速度设定值:%4s' % Set_rpm, '真实速度:%4s' % real_rpm, '电机占空比%4s' % pwm)
        #return pwm


#class PID():
    #def __init__(self, name):
        ## PID参数设置
        #self.kp, self.ki, self.kd = 0.6, 0.1, 0.05
        #self.Output_duty,self.error,self.lastErr0,self.last_last_Err0,self.pwm=0,0,0,0,0
        #self.name = name

    #def run(self, Set_rpm, real_rpm):
        #error = Set_rpm - real_rpm
        #dErr0 = self.error - self.lastErr0
        #self.Output_duty = int(self.kp * dErr0 + self.ki * error + self.kd * (error - 2 * self.lastErr0 + self.last_last_Err0))
        #self.pwm += self.Output_duty
        #self.lastErr0 = self.error
        #self.last_last_Err0 = self.lastErr0
        #if (self.pwm > 255):
            #self.pwm = 255
        #if (self.pwm < -255):
            #self.pwm = -255
        ##print('%s' % self.name, '速度设定值:%4s' % Set_rpm, '真实速度:%4s' % real_rpm, '电机占空比%4s' % pwm)
        #return self.pwm

def speed(turn_info=0, v0=100, det_v=150):
    global rpm0, rpm1
    if (turn_info < 0):
        zuopwm = zuopid.run(v0, rpm0)
        youpwm = youpid.run(v0 - turn_info * det_v, rpm1)
    else:
        zuopwm = zuopid.run(v0 + turn_info * det_v, rpm0)
        youpwm = youpid.run(v0, rpm1)
    to_arduino(zuopwm, youpwm)

def timer_speed(self):
    v0=150
    det_v=100
    global rpm0, rpm1,turn_info
    if (turn_info < 0):
        zuopwm = zuopid.run(v0, rpm0)
        youpwm = youpid.run(v0 - turn_info * det_v, rpm1)
    else:
        zuopwm = zuopid.run(v0 + turn_info * det_v, rpm0)
        youpwm = youpid.run(v0, rpm1)
    to_arduino(zuopwm, youpwm)



def to_arduino(zuopwm, youpwm):
    data = 'a'
    if (zuopwm < 0):
        data += '%03d' % -zuopwm + "000"
    else:
        data += "000" + '%03d' % zuopwm
    if (youpwm < 0):
        data += '%03d' % -youpwm + "000"
    else:
        data += "000" + '%03d' % youpwm
    data += "00b"
    uart1.write(data)


zuopid = PID("左轮")
youpid = PID("右轮")


def stop():
    uart1.write("a00000000000000b")


def getball():
    uart1.write("a00000000000010b")


def putball():
    uart1.write("a00000000000001b")


timer = Timer(Timer.TIMER1, Timer.CHANNEL0, mode=Timer.MODE_PERIODIC, period=50, callback=timer_speed)


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
    print(number)


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
    #img = sensor.snapshot().to_grayscale().morph(kernel_size, kernel).binary([(100,255)]).erode(1, threshold = 2)
    #img.morph(kernel_size, kernel)
    #img.binary([(60,145)])
    #img.erode(1, threshold = 2)
    #line = img.get_regression([(100,255)], roi=ROI,robust = True)

    img1 = sensor.snapshot()  # 正常读图片，为了找绿色做处理
    img1 = img1.resize(32,32)
    green_blobs = img1.find_blobs([green], merge=True)  # 所有的绿色块
    green_pixels = 0
    if (len(green_blobs)):
        for green_blob in green_blobs:
            green_pixels += green_blob.pixels()
    print("绿色：",green_pixels)
    #if(line):
        #rho_err = (abs(line.rho())-img.width()/2) / 56
        ##print(rho_err,line.magnitude(),rho_err)
        #return rho_err
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
            obs.append([i.classid(), i.value(), i.y() - i.h(), (i.x() + i.w() / 2 - 112)])
    return obs


# ——————————————————————————————————————————————
# ——————————————————————————————————————————————
# ——————————————————————————————————————————————
# ——————————————————————————————————————————————

timer.start()

while(1):
    obs = yolo()
    for i in range(len(obs)):
        if (obs[i][0] == football):
            turn_info = obs[i][3] / 10
            print("识别结果：", obs, turn_info)


# ————————————————————————————————————————
# 等待用户下达开始命令
# ————————————————————————————————————————
direction = listen()

# ————————————————————————————————————————
# 调整初始方向
# ————————————————————————————————————————
#提供一个初始速度，跑起来
for i in range(4):
    uart1.write("a00025500025500b")
    utime.sleep_ms(100)

while (1):
    dif = direction - mag
    print("磁场差值:", dif)
    if (dif < 2 and dif > -2):
        break
    elif(dif>0):
        uart1.write("a00025500000000b")
    else:
        uart1.write("a00000000025500b")

#等待1秒
for i in range(10):
    stop()
    utime.sleep_ms(100)

# ————————————————————————————————————————
# 识别是否进入跑道
# ————————————————————————————————————————


# ——————————————————————————————————————————
# 目标检测、避障
# ——————————————————————————————————————————
is_football=Flase
while(1):
    obs = yolo()
    for i in range(len(obs)):
        if (obs[i][0] == football):
            is_football=True
            break
    if(is_football):
        break

while (1):
    # 测距
    dist = distance()
    print(dist)
    if (dist <= 5):
        getball()
        break
    # 目标检测

    obs = yolo()
    for i in range(len(obs)):
        if (obs[i][0] == football):
            turn_info = obs[i][3] / 112
            print("识别结果：", obs, turn_info)
            break
    speed(turn_info)

# ————————————————————————————————————————————
# 寻找足球门框，目标检测
# ————————————————————————————————————————————
flag = False
while (1):
    speed(-1)
    obs = yolo()
    for i in range(len(obs)):
        if (obs[i][0] == goal):
            stop()
            flag = True
            break
    if (flag):
        break

while (1):
    dist = distance()
    if (dist <= 5):
        getball()
        break
    speed(0)
# ————————————————————————————————————————————
# 走向足球门框，目标检测，避障
# ————————————————————————————————————————————


# ————————————————————————————————————————————
# 放球
# ————————————————————————————————————————————
putball()

print("结束任务")




# 测试帧率
# t = time.ticks_ms()
# t = time.ticks_ms() - t
# print(t)


# 显示堆内存
# print(Maix.utils.heap_free() / 1024)
# 显示栈内存
# print(gc.mem_free() / 1024)

