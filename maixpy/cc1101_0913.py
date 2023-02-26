import time, network, utime
from Maix import GPIO
from machine import SPI
from fpioa_manager import fm

spi = SPI(id=0, mode=SPI.MODE_MASTER, baudrate=9600, polarity=0, phase=0, bits=8, firstbit=SPI.MSB, sck=10, mosi=15,miso=17, cs0=11)


CC1101_FREQ2 = 0x0D  # Frequency control word, high INT8U
CC1101_FREQ1 = 0x0E  # Frequency control word, middle INT8U
CC1101_FREQ0 = 0x0F  # Frequency control word, low INT8U
def SpiWriteReg(reg, value):
  spi.write(reg, cs=SPI.CS0)
  spi.write(value, cs=SPI.CS0)


def setMHZ():
    SpiWriteReg(bytearray(b'0x0D'), bytearray(b'\x00\x00\x10'))
    SpiWriteReg(bytearray(b'0x0E'), bytearray(b'\x00\x00\xb0'))
    SpiWriteReg(bytearray(b'0x0F'), bytearray(b'\x00\x00\x71'))
    print("初始化")


while(1):
    setMHZ()
    for i in range(10):
        data = spi.read(10, cs=SPI.CS0)
        print(data)
        utime.sleep(1)

