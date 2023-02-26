//New transmission method.
//In addition, the gdo0 and gdo2 pin are not required.
//https://github.com/LSatan/SmartRC-CC1101-Driver-Lib
//by Little_S@tan
#include <ELECHOUSE_CC1101_SRC_DRV.h>

//const int n = 6;
//char buffer[n] = "";
//byte buffer[n] = "";
byte data[6] = { 100, 100, 100, 100, 100, 100 };

int value = 0;

void setup() {

  Serial.begin(9600);

  if (ELECHOUSE_cc1101.getCC1101()) {  // Check the CC1101 Spi connection.
    Serial.println("Connection OK");
  } else {
    Serial.println("Connection Error");
  }

  ELECHOUSE_cc1101.Init();            // must be set to initialize the cc1101!
  ELECHOUSE_cc1101.setCCMode(1);      // set config for internal transmission mode.
  ELECHOUSE_cc1101.setModulation(0);  // set modulation mode. 0 = 2-FSK, 1 = GFSK, 2 = ASK/OOK, 3 = 4-FSK, 4 = MSK.
  ELECHOUSE_cc1101.setMHZ(433.92);    // Here you can set your basic frequency. The lib calculates the frequency automatically (default = 433.92).The cc1101 can: 300-348 MHZ, 387-464MHZ and 779-928MHZ. Read More info from datasheet.
  ELECHOUSE_cc1101.setSyncMode(2);    // Combined sync-word qualifier mode. 0 = No preamble/sync. 1 = 16 sync word bits detected. 2 = 16/16 sync word bits detected. 3 = 30/32 sync word bits detected. 4 = No preamble/sync, carrier-sense above threshold. 5 = 15/16 + carrier-sense above threshold. 6 = 16/16 + carrier-sense above threshold. 7 = 30/32 + carrier-sense above threshold.
                                      // ELECHOUSE_cc1101.setPA(10);      // set TxPower. The following settings are possible depending on the frequency band.  (-30  -20  -15  -10  -6    0    5    7    10   11   12) Default is max!
  ELECHOUSE_cc1101.setCrc(1);         // 1 = CRC calculation in TX and CRC check in RX enabled. 0 = CRC disabled for TX and RX.

  Serial.println("Tx Mode");

  pinMode(2, INPUT_PULLUP);  //上拉电阻保持电平稳定
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);

}



void loop() {
  // value = analogRead(A0);
  // Serial.print("X:");
  // Serial.print(value);
  // value = analogRead(A1);
  // Serial.print(" | Y:");
  // Serial.print(value);
  // value = digitalRead(2);
  // Serial.print(" | Z: ");
  // Serial.println(value);
  // delay(100);


  // buffer[0]=char(0);
  // buffer[1]=char(100);
  // buffer[2]=char(0);
  // buffer[3]=char(100);
  // buffer[4]=char(0);
  // buffer[5]=char(0);
  // ELECHOUSE_cc1101.SendData(buffer);
  // delay(1000);


  // if (Serial.available()) {
  //   int len = Serial.readBytesUntil('\n', buffer, n);
  //   buffer[len] = '\0';
  //   Serial.println((char *)buffer);
  //   ELECHOUSE_cc1101.SendData(buffer, len, 100);
  // }
  // ELECHOUSE_cc1101.SendData(data, 6, 100);
  // delay(1000);
ELECHOUSE_cc1101.SendData(data, 6, 100);
delay(1000);
ELECHOUSE_cc1101.SendData("Hello World", 100);
delay(1000);
Serial.println("OK");
}