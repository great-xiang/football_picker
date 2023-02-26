import time, network, utime
from Maix import GPIO
from machine import SPI
from fpioa_manager import fm


class CC1101():
    CC1101_IOCFG2 = 0x00  # GDO2 output pin configuration
    CC1101_IOCFG0 = 0x02  # GDO0 output pin configuration
    CC1101_PKTLEN = 0x06  # Packet length
    CC1101_PKTCTRL1 = 0x07  # Packet automation control
    CC1101_PKTCTRL0 = 0x08  # # Packet automation control
    CC1101_ADDR = 0x09  # Device address
    CC1101_CHANNR = 0x0A  # Channel number
    CC1101_FSCTRL1 = 0x0B  # Frequency synthesizer control
    CC1101_FSCTRL0 = 0x0C  # Frequency synthesizer control
    CC1101_FREQ2 = 0x0D  # Frequency control word, high INT8U
    CC1101_FREQ1 = 0x0E  # Frequency control word, middle INT8U
    CC1101_FREQ0 = 0x0F  # Frequency control word, low INT8U
    CC1101_MDMCFG4 = 0x10  # Modem configuration
    CC1101_MDMCFG3 = 0x11  # # Modem configuration
    CC1101_MDMCFG2 = 0x12  # Modem configuration
    CC1101_MDMCFG1 = 0x13  # Modem configuration
    CC1101_MDMCFG0 = 0x14  # Modem configuration
    CC1101_DEVIATN = 0x15  # Modem deviation setting
    CC1101_MCSM0 = 0x18  # Main Radio Control State Machine configuration
    CC1101_FOCCFG = 0x19  # Frequency Offset Compensation configuration
    CC1101_BSCFG = 0x1A  # Bit Synchronization configuration
    CC1101_AGCCTRL2 = 0x1B  # AGC control
    CC1101_AGCCTRL1 = 0x1C  # AGC control
    CC1101_AGCCTRL0 = 0x1D  # AGC control
    CC1101_FREND1 = 0x21  # Front end RX configuration
    CC1101_FREND0 = 0x22  # Front end TX configuration
    CC1101_FSCAL3 = 0x23  # Frequency synthesizer calibration
    CC1101_FSCAL2 = 0x24  # Frequency synthesizer calibration
    CC1101_FSCAL1 = 0x25  # Frequency synthesizer calibration
    CC1101_FSCAL0 = 0x26  # Frequency synthesizer calibration
    CC1101_FSTEST = 0x29  # Frequency synthesizer calibration control
    CC1101_TEST2 = 0x2C  # Various test settings
    CC1101_TEST1 = 0x2D  # Various test settings
    CC1101_TEST0 = 0x2E  # Various test settings
    CC1101_PATABLE = 0x3E
    WRITE_BURST = 0x40
    READ_BURST = 0xC0
    MHz = 433.92
    chan = 0
    pa = 12
    clb2 = [31, 38]
    PA_TABLE_433 = [0x12, 0x0E, 0x1D, 0x34, 0x60, 0x84, 0xC8, 0xC0]
    PA_TABLE = [0x00, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    m4RxBw = 0
    m2DCOFF = 0
    m2MANCH = 0
    m2SYNCM = 0
    m2MODFM = 0
    pc1PQT = 0
    pc1CRC_AF = 0
    pc1APP_ST = 0
    pc1ADRCHK = 0
    c0WDATA = 0
    pc0PktForm = 0
    pc0LenConf = 0
    pc0WDATA = 0
    CC1101_RXBYTES = 0x3B
    BYTES_IN_RXFIFO = 0x7F
    CC1101_RXFIFO = 0x3F
    CC1101_SRX = 0x34
    READ_SINGLE = 0x80
    CC1101_SFRX = 0x3A
    CC1101_SRES = 0x30
    CC1101_RSSI = 0x34
    CC1101_LQI = 0x33

    def __init__(self):
        # polarity： 极性， 取值为 0 或 1， 表示 SPI 在空闲时的极性， 0 代表低电平， 1 代表高电平
        # phase： 相， 取值位 0 或 1， 表示在时钟的第一个还是第二个跳变沿采集数据， 0 表示第一个， 1 表示第二个
        # 默认pol=1，pha=0,id=1,波特率5000000
        self.spi = SPI(id=1, mode=SPI.MODE_MASTER, baudrate=100000, polarity=0, phase=0, firstbit=SPI.MSB, sck=15,
                       mosi=11, miso=10, cs0=17)
        # 重置cc1101
        self.spi.write(self.CC1101_SRES)
        # 设置寄存器
        self.RegConfigSettings()
        # 设置同步字检测模式
        self.setSyncMode()
        # 设置CRC校验
        self.setCrc()

    def RegConfigSettings(self):
        self.SpiWriteReg(self.CC1101_FSCTRL1, 0x06)

        self.setCCMode()
        self.setMHZ()

        self.SpiWriteReg(self.CC1101_MDMCFG1, 0x02)
        self.SpiWriteReg(self.CC1101_MDMCFG0, 0xF8)
        self.SpiWriteReg(self.CC1101_CHANNR, self.chan)
        self.SpiWriteReg(self.CC1101_DEVIATN, 0x47)
        self.SpiWriteReg(self.CC1101_FREND1, 0x56)
        self.SpiWriteReg(self.CC1101_MCSM0, 0x18)
        self.SpiWriteReg(self.CC1101_FOCCFG, 0x16)
        self.SpiWriteReg(self.CC1101_BSCFG, 0x1C)
        self.SpiWriteReg(self.CC1101_AGCCTRL2, 0xC7)
        self.SpiWriteReg(self.CC1101_AGCCTRL1, 0x00)
        self.SpiWriteReg(self.CC1101_AGCCTRL0, 0xB2)
        self.SpiWriteReg(self.CC1101_FSCAL3, 0xE9)
        self.SpiWriteReg(self.CC1101_FSCAL2, 0x2A)
        self.SpiWriteReg(self.CC1101_FSCAL1, 0x00)
        self.SpiWriteReg(self.CC1101_FSCAL0, 0x1F)
        self.SpiWriteReg(self.CC1101_FSTEST, 0x59)
        self.SpiWriteReg(self.CC1101_TEST2, 0x81)
        self.SpiWriteReg(self.CC1101_TEST1, 0x35)
        self.SpiWriteReg(self.CC1101_TEST0, 0x09)
        self.SpiWriteReg(self.CC1101_PKTCTRL1, 0x04)
        self.SpiWriteReg(self.CC1101_ADDR, 0x00)
        self.SpiWriteReg(self.CC1101_PKTLEN, 0x00)

    def write(self, addr):
        self.spi.write(addr)

    def SpiReadStatus(self, addr):
        temp = addr | self.READ_BURST
        self.spi.write(temp)
        value = int.from_bytes(self.spi.read(1), 'big')
        return value

    def SpiReadReg(self, addr):
        temp = addr | self.READ_SINGLE
        self.spi.write(temp)
        value = int.from_bytes(self.spi.read(1), 'big')
        return value

    def SpiWriteReg(self, addr, value):
        self.spi.write(addr)
        self.spi.write(value)

    def SpiWriteBurstReg(self, addr, buffer, num):
        temp = addr | self.WRITE_BURST
        self.spi.write(temp)
        for i in range(num):
            self.spi.write(buffer[i])

    def SpiReadBurstReg(self, addr, num):
        temp = addr | self.READ_BURST
        buffer = []
        self.spi.write(temp)
        buffer = self.spi.read(num)
        return buffer

    def SpiStrobe(self, strobe):
        self.spi.write(strobe)

    def getCC1101(self):
        if (self.SpiReadStatus(0x31) > 0):
            print("连接正常")
        else:
            print("连接错误")

    def setCCMode(self):
        self.SpiWriteReg(self.CC1101_IOCFG2, 0x0B)
        self.SpiWriteReg(self.CC1101_IOCFG0, 0x06)
        self.SpiWriteReg(self.CC1101_PKTCTRL0, 0x05)
        self.SpiWriteReg(self.CC1101_MDMCFG3, 0xF8)
        self.SpiWriteReg(self.CC1101_MDMCFG4, 11 + self.m4RxBw)

        self.setModulation()

    # modulation默认为0
    def setModulation(self):
        global m2DCOFF, m2MANCH, m2SYNCM
        self.Split_MDMCFG2()

        self.m2MODFM = 0x00
        self.frend0 = 0x10
        self.SpiWriteReg(self.CC1101_MDMCFG2, self.m2DCOFF + self.m2MODFM + self.m2MANCH + self.m2SYNCM)
        self.SpiWriteReg(self.CC1101_FREND0, self.frend0)

        # 设置发射功率功率，默认
        self.setPA()

    # 拆分数据
    def Split_MDMCFG2(self):
        calc = self.SpiReadStatus(18)

        self.m2DCOFF = 0
        self.m2MODFM = 0
        self.m2MANCH = 0
        self.m2SYNCM = 0
        i = False
        while (i):
            if (calc >= 128):
                calc -= 128
                self.m2DCOFF += 128

            elif (calc >= 16):
                calc -= 16
                self.m2MODFM += 16

            elif (calc >= 8):
                calc -= 8
                self.m2MANCH += 8
            else:
                self.m2SYNCM = calc
                i = True

    def setPA(self):
        a = self.PA_TABLE_433[7]
        self.PA_TABLE[0] = a
        self.PA_TABLE[1] = 0
        self.SpiWriteBurstReg(self.CC1101_PATABLE, self.PA_TABLE, 8)

    def setMHZ(self):
        self.SpiWriteReg(self.CC1101_FREQ2, 16)
        self.SpiWriteReg(self.CC1101_FREQ1, 176)
        self.SpiWriteReg(self.CC1101_FREQ0, 113)
        self.Calibrate()

    def setSyncMode(self):
        self.Split_MDMCFG2()
        self.m2SYNCM = 0
        self.m2SYNCM = 2
        self.SpiWriteReg(self.CC1101_MDMCFG2, self.m2DCOFF + self.m2MODFM + self.m2MANCH + self.m2SYNCM)

    def setCrc(self):
        self.Split_PKTCTRL0()
        pc0CRC_EN = 0
        pc0CRC_EN = 4
        self.SpiWriteReg(self.CC1101_PKTCTRL0, self.pc0WDATA + self.pc0PktForm + self.pc0CRC_EN + self.pc0LenConf)

    def Split_PKTCTRL0(self):

        calc = self.SpiReadStatus(8)
        self.pc0WDATA = 0
        self.pc0PktForm = 0
        self.pc0CRC_EN = 0
        self.pc0LenConf = 0
        i = False
        while (i):

            if (calc >= 64):

                calc -= 64
                self.pc0WDATA += 64

            elif (calc >= 16):
                calc -= 16
                self.pc0PktForm += 16
            elif (calc >= 4):
                calc -= 4
                self.pc0CRC_EN += 4
            else:
                self.pc0LenConf = calc
                i = False

    def getRssi(self):
        rssi = self.SpiReadStatus(self.CC1101_RSSI)
        if (rssi >= 128):
            rssi = (rssi - 256) / 2 - 74
        else:
            rssi = (rssi / 2) - 74
        return rssi

    def getLqi(self):
        lqi = self.SpiReadStatus(self.CC1101_LQI)
        return lqi

    def Calibrate(self):
        self.SpiWriteReg(self.CC1101_FSCTRL0, 35)
        self.SpiWriteReg(self.CC1101_TEST0, 0x09)
        s = self.SpiReadStatus(self.CC1101_FSCAL2)
        if (s < 32):
            self.SpiWriteReg(self.CC1101_FSCAL2, s + 32)

    def ReceiveData(self):
        if (self.SpiReadStatus(self.CC1101_RXBYTES)):
            size = self.SpiReadReg(self.CC1101_RXFIFO)
            print("待读取数据位数:", size)
            rxBuffer = self.SpiReadBurstReg(self.CC1101_RXFIFO, size)
            #status = self.SpiReadBurstReg(self.CC1101_RXFIFO, 2)
            #print("状态：", status)
            #清空RX缓冲区
            self.SpiStrobe(self.CC1101_SFRX)
            #RX使能
            self.SpiStrobe(self.CC1101_SRX)
            return rxBuffer
        else:
            #清空RX缓冲区
            self.SpiStrobe(self.CC1101_SFRX)
            #RX使能
            self.SpiStrobe(self.CC1101_SRX)
            return 0

    def CheckRxFifo(self, t):
        while (1):
            if (self.SpiReadReg(self.CC1101_RXBYTES) & self.BYTES_IN_RXFIFO):
                return 1
            time.sleep(t / 1000)


# 实例化类
cc1101 = CC1101()

while (1):
    #cc1101.getCC1101()
    # LQI(链路质量指示)和RSSI(接收信号强度指示)
    #print("Rssi", cc1101.getRssi())
    #print("Lqi", cc1101.getLqi())
    #cc1101.CheckRxFifo(100)
    print(cc1101.ReceiveData())
    print()
    time.sleep(1)
