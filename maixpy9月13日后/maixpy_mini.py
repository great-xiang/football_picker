import image, utime, time, gc, sensor,Maix,sys,random
from fpioa_manager import fm
from Maix import GPIO, I2S
from machine import Timer, UART
import KPU as kpu


# ——————————————————————————————————————————————————————————————————————————————————————————
# 通信接口设置Uart，共有三个，1~3，uart UART 号，波特率,UART 数据宽度，奇偶校验位，停止位,串口接收超时时间,串口接收缓冲
# ——————————————————————————————————————————————————————————————————————————————————————————
#连接arduino模块
fm.register(0, fm.fpioa.UART1_TX, force=True)
fm.register(1, fm.fpioa.UART1_RX, force=True)
uart1 = UART(UART.UART1, 9600, 8, 1, 0, timeout=1000, read_buf_len=4096)
# 语音识别uart
fm.register(2, fm.fpioa.UART2_TX, force=True)
fm.register(3, fm.fpioa.UART2_RX, force=True)
uart2 = UART(UART.UART2, 9600, 8, 1, 0, timeout=1000, read_buf_len=4096)
#连接无线通讯模块，esp32,34RX，35TX
fm.register(10, fm.fpioa.UART3_TX, force=True)
fm.register(11, fm.fpioa.UART3_RX, force=True)
uart3 = UART(UART.UART3, 115200, 8, None, 0, timeout=1000, read_buf_len=4096)
# 定时器回调函数
def arduino(self):
    global mag, rpm0, rpm1
    read_data = uart1.read()
    if (read_data != None):
        #print(read_data)
        data = str(read_data)[2:-1]
        data = data.split(',')
        try:
            data = data[data.index("a") + 1:data.index("b")]
            mag = int(data[0])
            rpm0, rpm1 = 4*int(data[1]), 4*int(data[2])
            print('磁场度数:%3s' % data[0], '真实速度:%4s' % rpm0, '%4s' % rpm1, 'pwm%3s' % data[3], '%3s' % data[4], '%3s' % data[5], '%3s' % data[6])
        except Exception as e:
            # 访问异常的错误编号和详细信息
            print(e)



# 与arduino通信，100ms自动通信一次
tim = Timer(Timer.TIMER0, Timer.CHANNEL0, mode=Timer.MODE_PERIODIC, period=10, callback=arduino)
tim.start()

def cc1101():
    while (1):
        data = uart3.read()
        if (data != None):
            data = str(data)
            print("无线接收数据：",data)
            if(data[2]=="a" and data[-2]=="b"):
                return data[2:-1]
        utime.sleep_ms(100)
#while(1):
    #uart1.write(cc1101())
def to_arduino(zuopwm, youpwm):
    data = ""
    if (zuopwm < 0):
        data += chr(-zuopwm) + chr(0)
    else:
        data += chr(0) + chr(zuopwm)
    if (youpwm < 0):
        data += chr(-youpwm) + chr(0)
    else:
        data += chr(0) + chr(youpwm)
    data += chr(0)+chr(0)
    uart1.write(data)
while(1):
    to_arduino(100,100)
    print("发送一次")
    utime.sleep_ms(100)
