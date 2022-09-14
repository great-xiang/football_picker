#include <MsTimer2.h> //定时器库的头文件
#include <Wire.h>     //I2C Arduino Library
#include <math.h>
#define addr 0x1E // HMC5883的I2C地址,scl A5 ,sda A4

int angle, pwm[4], i, temp = 0;

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
    //定时器，每 50ms触发一次，调用 read_speed（）函数
    MsTimer2::set(50, read_speed);
    MsTimer2::start();

    //电机编码器反馈信号
    attachInterrupt(digitalPinToInterrupt(left0), zuolun, FALLING);  // 2脚下降沿触发中断
    attachInterrupt(digitalPinToInterrupt(right1), youlun, FALLING); // 3脚下降沿触发中断

    // serial通信设置
    Serial.begin(115200);

    // HMC5883磁场编码器初始化
    Wire.begin();
    Wire.beginTransmission(addr);
    Wire.write(0x02); //设置寄存器
    Wire.write(0x00); //设置连续测量模式
    Wire.endTransmission();
}
//主循环函数
void loop()
{
    maganet();
    Serial.print("a");
    Serial.print(",");
    Serial.print(angle, DEC);
    Serial.print(",");
    Serial.print(rpm0, DEC);
    Serial.print(",");
    Serial.print(rpm1, DEC);
    Serial.print(",");
    for (i = 0; i < 4; i++)
    {
        Serial.print(pwm[i]);
        Serial.print(",");
    }
    Serial.print("b");
    Serial.print(",");
    control();
    delay(1);
}

//读取UART缓冲区，读取速度。
void control()
{
    while (1)
    { //检查串口缓存中是否有16位数据等待读取
        if (Serial.available() >= 16)
        {
            //判断起始位是否是a
            if (Serial.read() != 97)
            {
                //清空缓冲区中的数据
                while (Serial.available() != 0)
                {
                    Serial.read();
                }
                return;
            }
            //读取串口缓存中等待的字符,并赋值给字符变量pwm
            for (i = 0; i < 4; i++)
            {
                pwm[i] = (Serial.read() - 48) * 100;
                pwm[i] += (Serial.read() - 48) * 10;
                pwm[i] += (Serial.read() - 48);
            }
            int trig0 = Serial.read() - 48;
            int trig1 = Serial.read() - 48;
            //判断结束位是否是b
            if (Serial.read() != 98)
            {
                //清空缓冲区中的数据
                while (Serial.available() != 0)
                {
                    Serial.read();
                }
                return;
            }

            if (pwm[0] != 0)
            {
                analogWrite(zuohou, pwm[0]);
            }
            else if (pwm[1] != 0)
            {
                analogWrite(zuoqian, pwm[1]);
            }
            else
            {
                analogWrite(zuohou, 0);
                analogWrite(zuoqian, 0);
            }
            if (pwm[2] != 0)
            {
                analogWrite(youhou, pwm[2]);
            }
            else if (pwm[3] != 0)
            {
                analogWrite(youqian, pwm[3]);
            }
            else
            {
                analogWrite(youhou, 0);
                analogWrite(youqian, 0);
            }
            if (trig0)
            {
                trig0 = getball();
                while (1)
                {
                    if (trig0 == 0)
                    {
                        break;
                    }
                    delay(10);
                }
            }
            if (trig1)
            {
                trig1 = putball();
                while (1)
                {
                    if (trig1 == 0)
                    {
                        break;
                    }
                    delay(10);
                }
            }
            break;
        }

        delay(10);
    }
}

//读取磁场
void maganet()
{
    //西0°，北130°，东195°，南256°
    int x, y, z; // triple axis data
    Wire.beginTransmission(addr);
    Wire.write(0x03); // start with register 3.
    Wire.endTransmission();

    // Read the data.. 2 bytes for each axis.. 6 total bytes
    Wire.requestFrom(addr, 6);
    if (6 <= Wire.available())
    {
        x = Wire.read() << 8; // MSB  x
        x |= Wire.read();     // LSB  x
        z = Wire.read() << 8; // MSB  z
        z |= Wire.read();     // LSB z
        y = Wire.read() << 8; // MSB y
        y |= Wire.read();     // LSB y
    }

    angle = (int(atan2((double)y, (double)x) * (180 / 3.14159265) + 180)); // angle in degrees
    String angle_str = String(angle);
}

//产生速度函数
void read_speed()
{
    rpm0 = int(number0); //减速比21.3，编码器精度11
    number0 = 0;
    rpm1 = int(number1);
    number1 = 0;
}

//左轮编码器脉冲计数中断函数
void zuolun()
{
    if (digitalRead(left0) == LOW)
    {
        if (digitalRead(left1) == LOW) // 查询EN_B的电平以确认正转
        {
            number0--;
        }
        if (digitalRead(left1) == HIGH) // 查询EN_B的电平以确认反转
        {
            number0++;
        }
    }
}
//右轮编码器脉冲计数中断函数
void youlun()
{
    if (digitalRead(right1) == LOW)
    {
        if (digitalRead(right0) == LOW) // 查询EN_B的电平以确认正转
        {
            number1++;
        }
        if (digitalRead(right0) == HIGH) // 查询EN_B的电平以确认反转
        {
            number1--;
        }
    }
}

//抓球
int getball()
{
    digitalWrite(zhouqian, HIGH);
    digitalWrite(zhouhou, LOW);
    while (1)
    {
        if (digitalRead(zhou0) != temp)
        {
            temp = digitalRead(zhou0);
            number3++;
        };
        if (number3 == num)
        {
            number3 = 0;
            digitalWrite(zhouqian, 0);
            digitalWrite(zhouhou, 0);
            return 0;
        };
    }
}

//放球
int putball()
{
    digitalWrite(zhouqian, LOW);
    digitalWrite(zhouhou, HIGH);
    while (1)
    {
        if (digitalRead(zhou0) != temp)
        {
            temp = digitalRead(zhou0);
            number3++;
        };
        if (number3 == num)
        {
            number3 = 0;
            digitalWrite(zhouqian, 0);
            digitalWrite(zhouhou, 0);
            return 0;
        };
    }
}
