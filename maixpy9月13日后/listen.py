from fpioa_manager import fm
# 语音识别uart
fm.register(2, fm.fpioa.UART2_TX, force=True)
fm.register(3, fm.fpioa.UART2_RX, force=True)
uart2 = UART(UART.UART2, 9600, 8, 1, 0, timeout=1000, read_buf_len=4096)
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
