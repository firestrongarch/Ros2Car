#pragma once
#include <Arduino.h>

class Encoder{
public:
    struct Data{
        int count;
        int a,b;
        float current_vel;
    };
    struct Quaternion{
        float w;
        float x;
        float y;
        float z;
    };
    struct Odom{
        float x;                 // 坐标x
        float y;                 // 坐标y
        float yaw;               // yaw
        Quaternion quaternion;
        float linear_speed;      // 线速度
        float angular_speed;     // 角速度
    };

private:
    // 电机参数
    int reducation = 48;//减速比，根据电机参数设置，比如 15 | 30 | 60
    int pulse = 13; //编码器旋转一圈产生的脉冲数该值需要参考商家电机参数
    int per_round = pulse * reducation * 4;//车轮旋转一圈产生的脉冲数 
    long start_time = millis();//一个计算周期的开始时刻，初始值为 millis();
    long interval_time = 50;//一个计算周期 50ms
    float wheel_diameter = 0.068;
    float wheel_distance_ = 0.15; // 轮子间距

    static Data left,right;
    Odom odom_;
    
public:
    Encoder(const int left_a,const int left_b,const int right_a,const int right_b);
    static void count_left_a();
    static void count_left_b();
    static void count_right_a();
    static void count_right_b();
    void get_current_vel();
    float* get_pointer_left_vel();
    float* get_pointer_right_vel();

    void TransAngleInPI(float angle, float &out_angle){
        if (angle > PI){
            out_angle -= 2 * PI;
        }
        else if (angle < -PI){
            out_angle += 2 * PI;
        }
    }
    void kinematic_inverse(float linear_speed, float angular_speed, float &out_wheel1_speed, float &out_wheel2_speed){
        out_wheel1_speed = linear_speed - (angular_speed * wheel_distance_) / 2.0;
        out_wheel2_speed = linear_speed + (angular_speed * wheel_distance_) / 2.0;
    }
    void kinematic_forward(float wheel1_speed, float wheel2_speed, float &linear_speed, float &angular_speed){
        linear_speed = (wheel1_speed + wheel2_speed) / 2.0;   // 计算线速度
        angular_speed = (wheel2_speed - wheel1_speed) / wheel_distance_;   // 计算角速度
    }

    // 用于将欧拉角转换为四元数。
    void Euler2Quaternion(float roll, float pitch, float yaw, Quaternion &q){
        // 传入机器人的欧拉角 roll、pitch 和 yaw。
        // 计算欧拉角的 sin 和 cos 值，分别保存在 cr、sr、cy、sy、cp、sp 六个变量中    
        // https://blog.csdn.net/xiaoma_bk/article/details/79082629
        double cr = cos(roll * 0.5);
        double sr = sin(roll * 0.5);
        double cy = cos(yaw * 0.5);
        double sy = sin(yaw * 0.5);
        double cp = cos(pitch * 0.5);
        double sp = sin(pitch * 0.5);
        // 计算出四元数的四个分量 q.w、q.x、q.y、q.z
        q.w = cy * cp * cr + sy * sp * sr;
        q.x = cy * cp * sr - sy * sp * cr;
        q.y = sy * cp * sr + cy * sp * cr;
        q.z = sy * cp * cr - cy * sp * sr;
    }
    /**
     * @brief 更新机器人里程计信息
     * 
     * @param dt 间隔时间dt
     */
    void update_bot_odom(float dt_s){
        this->kinematic_forward(left.current_vel, right.current_vel, odom_.linear_speed, odom_.angular_speed);

        odom_.yaw += odom_.angular_speed * dt_s;

        TransAngleInPI(odom_.yaw, odom_.yaw);
        

        /*更新x和y轴上移动的距离*/
        float delta_distance = odom_.linear_speed * dt_s; // 单位m
        odom_.x += delta_distance * cos(odom_.yaw);
        odom_.y += delta_distance * sin(odom_.yaw);

        // 调用 Euler2Quaternion 函数，将机器人的欧拉角 yaw 转换为四元数 quaternion。
        Euler2Quaternion(0, 0, odom_.yaw, odom_.quaternion);
    }
    
    /**
     * @brief 获取里程计函数
     * 
     * @return odom_t& 
     */
    Odom &odom(){
        return odom_;
    }
};
