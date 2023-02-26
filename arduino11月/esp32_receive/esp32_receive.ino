//串口，GPIO16 is RXD2，GPIO17 is TXD2
#include <HardwareSerial.h>
//cc1101无线通讯
#include <ELECHOUSE_CC1101_SRC_DRV.h>
//看门狗
#include "soc/rtc_wdt.h"     //设置看门狗

HardwareSerial SerialPort(2);  // use UART2

void setup() {

  Serial.begin(9600);
  Serial2.begin(9600, SERIAL_8N1, 16, 17);

  if (ELECHOUSE_cc1101.getCC1101()) {  // Check the CC1101 Spi connection.
    Serial.println("Connection OK");
    Serial2.write("Connection OK");
  } else {
    Serial.println("Connection Error");
    Serial2.write("Connection Error");
  }

  ELECHOUSE_cc1101.Init();            // must be set to initialize the cc1101!
  ELECHOUSE_cc1101.setCCMode(1);      // set config for internal transmission mode.
  ELECHOUSE_cc1101.setModulation(0);  // set modulation mode. 0 = 2-FSK, 1 = GFSK, 2 = ASK/OOK, 3 = 4-FSK, 4 = MSK.
  ELECHOUSE_cc1101.setMHZ(433.92);    // Here you can set your basic frequency. The lib calculates the frequency automatically (default = 433.92).The cc1101 can: 300-348 MHZ, 387-464MHZ and 779-928MHZ. Read More info from datasheet.
  ELECHOUSE_cc1101.setSyncMode(2);    // Combined sync-word qualifier mode. 0 = No preamble/sync. 1 = 16 sync word bits detected. 2 = 16/16 sync word bits detected. 3 = 30/32 sync word bits detected. 4 = No preamble/sync, carrier-sense above threshold. 5 = 15/16 + carrier-sense above threshold. 6 = 16/16 + carrier-sense above threshold. 7 = 30/32 + carrier-sense above threshold.
  ELECHOUSE_cc1101.setCrc(1);         // 1 = CRC calculation in TX and CRC check in RX enabled. 0 = CRC disabled for TX and RX.



 //ESP32看门狗设置 需要先引入 #include "soc/rtc_wdt.h" //设置看门狗用
  rtc_wdt_protect_off();     //看门狗写保护关闭 关闭后可以喂狗
  //rtc_wdt_protect_on();    //看门狗写保护打开 打开后不能喂狗
  //rtc_wdt_disable();       //禁用看门狗
  rtc_wdt_enable();          //启用看门狗
  rtc_wdt_feed();            //喂狗
  rtc_wdt_set_time(RTC_WDT_STAGE0, 500);     // 设置看门狗超时 500ms.
  Serial2.println("chong qi");
}
byte buffer[61] = { 0 };

void loop() {
    if (ELECHOUSE_cc1101.CheckRxFifo(100)) {
      int len = ELECHOUSE_cc1101.ReceiveData(buffer);
      buffer[len] = '\0';
      Serial2.write((char *)buffer);
    }
    rtc_wdt_feed();//喂狗
}