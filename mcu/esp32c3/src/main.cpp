#include <Arduino.h>
#include "drv8833.h"
#include <micro_ros_platformio.h>
#include <WiFi.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>

#include <PID_v1.h> 
#include "encoder.h"
#include "kinematics.h"

// 定义 ROS2 执行器和支持结构
rclc_executor_t executor;
rclc_support_t support;
// 定义 ROS2 内存分配器
rcl_allocator_t allocator;
// 定义 ROS2 节点和订阅者
rcl_node_t node;
rcl_subscription_t subscriber;
// 定义接收到的消息结构体
geometry_msgs__msg__Twist sub_msg;

const int PWMA1 = 18;
const int PWMA2 = 19;
const int ENCAa = 2;  // right
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
drv8833 motors(PWMB1,PWMB2,PWMA1,PWMA2,eep);
Encoder encoder(ENCBa,ENCBb,ENCAa,ENCAb);

// drv8833电源
volatile byte state = LOW;

double left_pwm;//电机驱动的PWM值
double right_pwm;//电机驱动的PWM值
double target_left, target_right;
double kp=1.5, ki=3.0, kd=0.1;

PID pid_left(encoder.get_pointer_left_vel(),&left_pwm,&target_left,kp,ki,kd,DIRECT);
PID pid_right(encoder.get_pointer_right_vel(),&right_pwm,&target_right,kp,ki,kd,DIRECT);

//中断服务程序
void power() 
{
  state = !state;
}

Kinematics kinematics;           // 运动学相关对象
// 回调函数，当接收到新的 Twist 消息时会被调用
void twist_callback(const void *msg_in)
{
    const geometry_msgs__msg__Twist *twist_msg = (const geometry_msgs__msg__Twist *)msg_in;
    float linear_x = twist_msg->linear.x;   // 获取 Twist 消息的线性 x 分量
    float angular_z = twist_msg->angular.z; // 获取 Twist 消息的角度 z 分量
    kinematics.kinematic_inverse(linear_x, angular_z, target_left, target_right);
    if(linear_x == 0){
        motors.updateState(LOW);
    } else {
        motors.updateState(HIGH);
    }
    Serial.println("left & right vel:");
    Serial.println(target_left);
    Serial.println(target_right);
}

// 这个函数是一个后台任务，负责设置和处理与 micro-ROS 代理的通信。
void microros_task(void *param)
{
    // 设置 micro-ROS 代理的 IP 地址。
    IPAddress agent_ip;
    agent_ip.fromString("192.168.164.252");

    char ssid[] = "K50U";
    char pass[] = "88888888";

    // 使用 WiFi 网络和代理 IP 设置 micro-ROS 传输层。
    set_microros_wifi_transports(ssid, pass, agent_ip, 8888);

    // 等待 2 秒，以便网络连接得到建立。
    delay(2000);

    // 设置 micro-ROS 支持结构、节点和订阅。
    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "esp32_car", "", &support);
    rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/cmd_vel");

    // 设置 micro-ROS 执行器，并将订阅添加到其中。
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_subscription(&executor, &subscriber, &sub_msg, &twist_callback, ON_NEW_DATA);

    // 循环运行 micro-ROS 执行器以处理传入的消息。
    while (true){
        delay(100);
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
}

void setup()
{
    // 初始化串口
    Serial.begin(115200);

    // //将中断触发引脚（2号引脚）设置为INPUT_PULLUP（输入上拉）模式
    // pinMode(button, INPUT_PULLUP); 
    
    // //设置中断触发程序
    // attachInterrupt(digitalPinToInterrupt(button), power, FALLING);

    pid_left.SetMode(AUTOMATIC);
    pid_right.SetMode(AUTOMATIC);

    // pid_left.SetOutputLimits(-50, 50);
    // pid_right.SetOutputLimits(-50, 50);

    // 设置运动学参数
    kinematics.set_motor_param(0, 45, 44, 0.068);
    kinematics.set_motor_param(1, 45, 44, 0.068);
    kinematics.set_kinematic_param(0.15);

    // 在核心0上创建一个名为"microros_task"的任务，栈大小为10240
    xTaskCreatePinnedToCore(microros_task, "microros_task", 10240, NULL, 1, NULL, 0);
}

void loop()
{
    // motors.updateState(state);
    delay(10);
    encoder.get_current_vel();
    pid_left.Compute();//计算需要输出的PWM
    pid_right.Compute();
    if(target_left >= 0){
        motors.updateSpeed(1, left_pwm);
    } else {
        motors.updateSpeed(1, -left_pwm);
    }
    if(target_right >= 0){
        motors.updateSpeed(2, right_pwm);
    } else {
        motors.updateSpeed(2, -right_pwm);
    }
    // Serial.println("left_pwm:");
    // Serial.println(-left_pwm);
}
