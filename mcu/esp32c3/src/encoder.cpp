#include "encoder.h"
#include "esp32-hal-gpio.h"
#include <functional>
// #include "esp32-hal-gpio.h"

Encoder::Data Encoder::left{};
Encoder::Data Encoder::right{};
Encoder::Encoder(const int left_a,const int left_b,const int right_a,const int right_b) {
    left.a = left_a;
    left.b = left_b;

    right.a = right_a;
    right.b = right_b;
    pinMode(left_a, INPUT);
    pinMode(left_b, INPUT);
    attachInterrupt(digitalPinToInterrupt(left_a),count_left_a,CHANGE);
    attachInterrupt(digitalPinToInterrupt(left_b),count_left_b,CHANGE);

    pinMode(right_a, INPUT);
    pinMode(right_b, INPUT);
    attachInterrupt(digitalPinToInterrupt(right_a),count_right_a,CHANGE);
    attachInterrupt(digitalPinToInterrupt(right_b),count_right_b,CHANGE);
}

void Encoder::count_left_a(){
    //2倍频计数实现
    //手动旋转电机一圈，输出结果为 一圈脉冲数 * 减速比 * 2
    if(digitalRead(left.a) == HIGH){
        if(digitalRead(left.b) == HIGH){//A 高 B 高
            left.count++;  
        } else {//A 高 B 低
            left.count--;  
        }
    } else {
        if(digitalRead(left.b) == LOW){//A 低 B 低
            left.count++;  
        } else {//A 低 B 高
            left.count--;  
        }  
    }
}

void Encoder::count_left_b(){
    if(digitalRead(left.b) == HIGH){
        if(digitalRead(left.a) == LOW){//B 高 A 低
            left.count++;
        } else {//B 高 A 高
            left.count--;
        }
    } else {
        if(digitalRead(left.a) == HIGH){//B 低 A 高
            left.count++;
        } else {//B 低 A 低
            left.count--;
        }
    }
}

void Encoder::count_right_a(){
    //2倍频计数实现
    //手动旋转电机一圈，输出结果为 一圈脉冲数 * 减速比 * 2
    if(digitalRead(right.a) == HIGH){
        if(digitalRead(right.b) == HIGH){//A 高 B 高
            right.count++;  
        } else {//A 高 B 低
            right.count--;  
        }
    } else {
        if(digitalRead(right.b) == LOW){//A 低 B 低
            right.count++;  
        } else {//A 低 B 高
            right.count--;  
        }  
    }
}

void Encoder::count_right_b(){
    if(digitalRead(right.b) == HIGH){
        if(digitalRead(right.a) == LOW){//B 高 A 低
            right.count++;
        } else {//B 高 A 高
            right.count--;
        }
    } else {
        if(digitalRead(right.a) == HIGH){//B 低 A 高
            right.count++;
        } else {//B 低 A 低
            right.count--;
        }
    }
}

void Encoder::get_current_vel(){
    long right_now = millis();  
    long past_time = right_now - start_time;//计算逝去的时间
    if(past_time >= interval_time){//如果逝去时间大于等于一个计算周期
        //1.禁止中断
        noInterrupts();
        //2.计算转速 转速单位可以是秒，也可以是分钟... 自定义即可
        left.current_vel = (double)left.count / per_round / past_time * 1000 * 60;
        right.current_vel = (double)right.count / per_round / past_time * 1000 * 60;
        //3.重置计数器
        left.count = 0;
        right.count = 0;
        //4.重置开始时间
        start_time = right_now;
        //5.重启中断
        interrupts();
        Serial.println("left & right:");
        Serial.println(left.current_vel);
        Serial.println(right.current_vel);
    }
}