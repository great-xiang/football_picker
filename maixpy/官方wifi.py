import time, network,utime
from Maix import GPIO
from machine import UART
from fpioa_manager import fm
import socket
sock = socket.socket()
sock.close()

fm.register(7,fm.fpioa.UART2_TX)
fm.register(6,fm.fpioa.UART2_RX)
uart2 = UART(UART.UART2, 115200, timeout=1000, read_buf_len=8192)

#手机换成2.4G才能连接
class wifi():

    __is_m1w__ = True
    uart = None
    eb = None
    nic = None

    def init():
        if __class__.__is_m1w__:
            fm.register(0, fm.fpioa.GPIOHS1, force=True)
            M1wPower=GPIO(GPIO.GPIOHS1, GPIO.OUT)
            M1wPower.value(0) # b'\r\n ets Jan  8 2013,rst cause:1, boot mode:(7,6)\r\n\r\nwaiting for host\r\n'

        fm.register(8, fm.fpioa.GPIOHS0) # board_info.WIFI_EN == IO 8
        __class__.en = GPIO(GPIO.GPIOHS0,GPIO.OUT)

        fm.register(7,fm.fpioa.UART2_TX) # board_info.WIFI_RX == IO 7
        fm.register(6,fm.fpioa.UART2_RX) # board_info.WIFI_TX == IO 6
        __class__.uart = UART(UART.UART2, 115200, timeout=1000, read_buf_len=8192)

    def enable(en):
        __class__.en.value(en)

    def _at_cmd(cmd="AT\r\n", resp="OK\r\n", timeout=20):
        __class__.uart.write(cmd) # "AT+GMR\r\n"
        time.sleep_ms(timeout)
        tmp = __class__.uart.read()
        # print(tmp)
        if tmp and tmp.endswith(resp):
            return True
        return False

    def at_cmd(cmd="AT\r\n", timeout=20):
        __class__.uart.write(cmd) # "AT+GMR\r\n"
        time.sleep_ms(timeout)
        tmp = __class__.uart.read()
        return tmp

    def reset(force=False, reply=5):
        if force == False and __class__.isconnected():
            return True
        __class__.init()
        for i in range(reply):
            print('reset...')
            __class__.enable(False)
            time.sleep_ms(50)
            __class__.enable(True)
            time.sleep_ms(500) # at start > 500ms
            if __class__._at_cmd(timeout=500):
                break
        __class__._at_cmd()
        __class__._at_cmd('AT+UART_CUR=921600,8,1,0,0\r\n', "OK\r\n")
        __class__.uart = UART(UART.UART2, 921600, timeout=1000, read_buf_len=10240)
        # important! baudrate too low or read_buf_len too small will loose data
        #print(__class__._at_cmd())
        try:
            __class__.nic = network.ESP8285(__class__.uart)
            time.sleep_ms(500) # wait at ready to connect
        except Exception as e:
            print(e)
            return False
        return True

    def connect(ssid="wifi_name", pasw="pass_word"):
        if __class__.nic != None:
            return __class__.nic.connect(ssid, pasw)

    def ifconfig(): # should check ip != 0.0.0.0
        if __class__.nic != None:
            return __class__.nic.ifconfig()

    def isconnected():
        if __class__.nic != None:
            return __class__.nic.isconnected()
        return False


SSID = "xiang"#192.168.43.47
PASW = "33333333"

def check_wifi_net(reply=5):
    if wifi.isconnected() != True:
        for i in range(reply):
            try:
                wifi.reset()
                print('try AT connect wifi...', wifi._at_cmd())
                wifi.connect(SSID, PASW)
                if wifi.isconnected():
                    break
            except Exception as e:
                print(e)
    return wifi.isconnected()

if wifi.isconnected() == False:
    check_wifi_net()
print('network state:', wifi.isconnected(), wifi.ifconfig())

while(1):
    data=uart2.read()
    if(data!=None):
        print(data)
    utime.sleep(1)
