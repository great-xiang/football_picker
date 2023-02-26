import image, utime, time, gc, sensor,Maix,sys,random
from fpioa_manager import fm
from Maix import GPIO, I2S
from machine import Timer, UART
import KPU as kpu



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


# ——————————————————————————————————————————————————————————————————————————————————————————
# 通信接口设置Uart，共有三个，1~3，uart UART 号，波特率,UART 数据宽度，奇偶校验位，停止位,串口接收超时时间,串口接收缓冲
# ——————————————————————————————————————————————————————————————————————————————————————————
#连接arduino模块
fm.register(0, fm.fpioa.UART1_TX, force=True)
fm.register(1, fm.fpioa.UART1_RX, force=True)
uart1 = UART(UART.UART1, 9600, 8, None, 0, timeout=1000, read_buf_len=4096)
# 语音识别uart
fm.register(2, fm.fpioa.UART2_TX, force=True)
fm.register(3, fm.fpioa.UART2_RX, force=True)
uart2 = UART(UART.UART2, 9600, 8, 1, 0, timeout=1000, read_buf_len=4096)
#连接无线通讯模块，esp32,34RX，35TX
fm.register(0, fm.fpioa.UART1_TX, force=True)
fm.register(1, fm.fpioa.UART1_RX, force=True)
uart3 = UART(UART.UART3, 115200, 8, None, 0, timeout=1000, read_buf_len=4096)


# 测距模块
fm.register(8, fm.fpioa.GPIO0)  # trig触发信号
fm.register(9, fm.fpioa.GPIO1)  # echo回响信号
trig = GPIO(GPIO.GPIO0, GPIO.OUT)
echo = GPIO(GPIO.GPIO1, GPIO.IN)






def stop():
    uart1.write("a00000000000000b")

def go():
    uart1.write("a00018000025500b")

def getball():
    uart1.write("a00000000000010b")


def putball():
    uart1.write("a00000000000001b")


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

# ————————————————————————————————————————
# 等待用户下达开始命令
# ————————————————————————————————————————
listen()


for i in range(5):
    go()
    utime.sleep_ms(100)


while (1):
    dist = distance()
    if (dist <= 20):
        getball()
        break
    utime.sleep_ms(20)




for i in range(500):
    go()
    data = uart2.read()
    if (data != None):
        break
    utime.sleep_ms(100)

