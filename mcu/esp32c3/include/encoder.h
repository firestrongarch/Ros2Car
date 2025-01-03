#pragma once
#include <Arduino.h>

class Encoder{
public:
    struct Data{
        int count;
        int a,b;
        double current_vel;
    };
private:
    // 电机参数
    int reducation = 48;//减速比，根据电机参数设置，比如 15 | 30 | 60
    int pulse = 13; //编码器旋转一圈产生的脉冲数该值需要参考商家电机参数
    int per_round = pulse * reducation * 4;//车轮旋转一圈产生的脉冲数 
    long start_time = millis();//一个计算周期的开始时刻，初始值为 millis();
    long interval_time = 50;//一个计算周期 50ms
    float wheel_diameter = 0.068;

    static Data left,right;
    
public:
    Encoder(const int left_a,const int left_b,const int right_a,const int right_b);
    static void count_left_a();
    static void count_left_b();
    static void count_right_a();
    static void count_right_b();
    void get_current_vel();
    double* get_pointer_left_vel();
    double* get_pointer_right_vel();
};
