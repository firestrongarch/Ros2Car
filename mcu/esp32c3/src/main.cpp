#include <Arduino.h>
#include "drv8833.h"
#include <WiFi.h>

#include <PID_v1.h> 

const int PWMA1 = 18;
const int PWMA2 = 19;
const int ENCAa = 2;
const int ENCAb = 3;
int countA = 0; //如果是正转，那么每计数一次自增1，如果是反转，那么每计数一次自减1 

const int PWMB1 = 0;
const int PWMB2 = 1;
const int ENCBa = 10;
const int ENCBb = 6;
int countB = 0;

const int button = 9;
const int eep = 5;

// 定义控制两个电机的对象
drv8833 motors(PWMA1,PWMA2,PWMB1,PWMB2,eep);

// drv8833电源
volatile byte state = LOW;

//中断服务程序
void power() 
{
    state = !state;  
}

void setup()
{
    // 初始化串口
    Serial.begin(115200);

    //将中断触发引脚（2号引脚）设置为INPUT_PULLUP（输入上拉）模式
    pinMode(button, INPUT_PULLUP); 
    
    //设置中断触发程序
    attachInterrupt(digitalPinToInterrupt(button), power, FALLING);

}

void loop()
{
    motors.updateState(state);
    Serial.println("hello esp32");
    delay(10);
}
