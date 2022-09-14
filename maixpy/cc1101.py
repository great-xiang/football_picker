import time, network, utime
from Maix import GPIO
from machine import SPI
from fpioa_manager import fm

spi = SPI(0, mode=SPI.MODE_MASTER, baudrate=500000, polarity=0, phase=0, bits=8, firstbit=SPI.MSB, sck=10, mosi=15,
          miso=17, cs0=11)

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
MHz = 433.92
chan = 0
pa = 12
clb1 = {24, 28}
clb2 = {31, 38}
clb3 = {65, 76}
clb4 = {77, 79}
PA_TABLE_433 = [0x12, 0x0E, 0x1D, 0x34, 0x60, 0x84, 0xC8, 0xC0, ]


def SpiWriteReg(reg, value):
    spi.write(reg, cs=SPI.CS0)
    spi.write(value, cs=SPI.CS0)


PA_TABLE = [0, 0xc0]


def setpa():
    temp = bytes(CC1101_PATABLE) | WRITE_BURST
    spi.write(temp, cs=SPI.CS0)
    for i in range(8):
        SPI.transfer(PA_TABLE[i])


def setCCMode():
    SpiWriteReg(CC1101_IOCFG2, 0x0B)
    SpiWriteReg(CC1101_IOCFG0, 0x06)
    SpiWriteReg(CC1101_PKTCTRL0, 0x05)
    SpiWriteReg(CC1101_MDMCFG3, 0xF8)
    SpiWriteReg(CC1101_MDMCFG4, 11)


def setModulation():
    SpiWriteReg(CC1101_MDMCFG2, 0x00)
    SpiWriteReg(CC1101_FREND0, 0x10)


def setMHZ():
    byt1 = (16).to_bytes(3, 'big')
    byt2 = (176).to_bytes(3, 'big')
    byt3 = (113).to_bytes(3, 'big')
    SpiWriteReg(CC1101_FREQ2, byt1)
    SpiWriteReg(CC1101_FREQ1, byt2)
    SpiWriteReg(CC1101_FREQ0, byt3)

def RegConfigSettings():
    SpiWriteReg(CC1101_MDMCFG1, 0x02)
    SpiWriteReg(CC1101_MDMCFG0, 0xF8)
    SpiWriteReg(CC1101_CHANNR, 0)
    SpiWriteReg(CC1101_DEVIATN, 0x47)
    SpiWriteReg(CC1101_FREND1, 0x56)
    SpiWriteReg(CC1101_MCSM0, 0x18)
    SpiWriteReg(CC1101_FOCCFG, 0x16)
    SpiWriteReg(CC1101_BSCFG, 0x1C)
    SpiWriteReg(CC1101_AGCCTRL2, 0xC7)
    SpiWriteReg(CC1101_AGCCTRL1, 0x00)
    SpiWriteReg(CC1101_AGCCTRL0, 0xB2)
    SpiWriteReg(CC1101_FSCAL3, 0xE9)
    SpiWriteReg(CC1101_FSCAL2, 0x2A)
    SpiWriteReg(CC1101_FSCAL1, 0x00)
    SpiWriteReg(CC1101_FSCAL0, 0x1F)
    SpiWriteReg(CC1101_FSTEST, 0x59)
    SpiWriteReg(CC1101_TEST2, 0x81)
    SpiWriteReg(CC1101_TEST1, 0x35)
    SpiWriteReg(CC1101_TEST0, 0x09)
    SpiWriteReg(CC1101_PKTCTRL1, 0x04)
    SpiWriteReg(CC1101_ADDR, 0x00)
    SpiWriteReg(CC1101_PKTLEN, 0x00)


def setSyncMode():
    SpiWriteReg(CC1101_MDMCFG2, 2)


while (1):
    setCCMode()
    setModulation()
    setMHZ()
    RegConfigSettings()
    setSyncMode()
    for i in range(10):
        data = spi.read(10, write=0x00, cs=SPI.CS0)
        print(data)
        # spi.write(w, cs=SPI.CS0)
        utime.sleep(1)
