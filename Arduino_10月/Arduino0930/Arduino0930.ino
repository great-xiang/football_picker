#include <MsTimer2.h> //定时器库的头文件
#include <Wire.h>     //I2C Arduino Library
#include <math.h>
#define addr 0x1E // HMC5883的I2C地址,scl A5 ,sda A4

int angle, serialData[4], i, temp = 0;

//三个轮子，六根控制线
int zuoqian = 10;
int zuohou = 11;
int youqian = 6;
int youhou = 5;
int zhouqian = 9;
int zhouhou = 8;

//三个轮子，六根反馈线
int left0 = 2; //黄色
int left1 = 4; //绿色
int right0 = 7;
int right1 = 3;
int zhou0 = 12;
int zhou1 = 13;

// 电机反馈脉冲计数
long number0 = 0, number1 = 0, number3 = 0;
int rpm0 = 0, rpm1 = 0;
int num = 1100; //轴转的单位数



//初始化硬件设置
void setup()
{
    //引脚模式设置
    pinMode(left0, INPUT_PULLUP);
    pinMode(right0, INPUT_PULLUP);
    pinMode(left1, INPUT);
    pinMode(right1, INPUT);
    pinMode(zuoqian, OUTPUT);
    pinMode(zuohou, OUTPUT);
    pinMode(youqian, OUTPUT);
    pinMode(youhou, OUTPUT);
    pinMode(zhouqian, OUTPUT);
    pinMode(zhouhou, OUTPUT);


}
//主循环函数
void loop()
{


    analogWrite(zuoqian, 250);
    analogWrite(zuohou, 0);
    analogWrite(youqian, 250);
    analogWrite(youhou, 0);
    delay(2000);
        analogWrite(zuoqian, 0);
    analogWrite(zuohou, 250);
    analogWrite(youqian, 0);
    analogWrite(youhou, 250);
    delay(2000);
        analogWrite(zuoqian, 250);
    analogWrite(zuohou, 0);
    analogWrite(youqian, 0);
    analogWrite(youhou, 250);
    delay(2000);
        analogWrite(zuoqian, 0);
    analogWrite(zuohou, 250);
    analogWrite(youqian, 250);
    analogWrite(youhou, 0);
    delay(2000);
}
