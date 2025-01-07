#include <Arduino.h>
#include "HardwareSerial.h"
#include "drv8833.h"
#include <micro_ros_platformio.h>
#include <WiFi.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>

#include "encoder.h"
#include "rclc/publisher.h"

#include <QuickPID.h>

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

#include <nav_msgs/msg/odometry.h>
#include <micro_ros_utilities/string_utilities.h>
rcl_publisher_t odom_publisher;   // 用于发布机器人的里程计信息（Odom）
nav_msgs__msg__Odometry odom_msg; // 机器人的里程计信息
rcl_publisher_options_t publisher_options = rcl_publisher_get_default_options();

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
drv8833 motors(PWMA1,PWMA2,PWMB1,PWMB2,eep);
Encoder encoder(ENCBa,ENCBb,ENCAa,ENCAb);

// drv8833电源
volatile byte state = LOW;

float left_pwm = 0;//电机驱动的PWM值
float right_pwm = 0;//电机驱动的PWM值
float target_left, target_right;
float kp=2, ki=5, kd=1;
QuickPID pid_left(encoder.get_pointer_left_vel(),&left_pwm, &target_left);
QuickPID pid_right(encoder.get_pointer_right_vel(),&right_pwm, &target_right);
//中断服务程序
void power() 
{
  state = !state;
}

// 回调函数，当接收到新的 Twist 消息时会被调用
void twist_callback(const void *msg_in)
{
    const geometry_msgs__msg__Twist *twist_msg = (const geometry_msgs__msg__Twist *)msg_in;
    float linear_x = twist_msg->linear.x;   // 获取 Twist 消息的线性 x 分量
    float angular_z = twist_msg->angular.z; // 获取 Twist 消息的角度 z 分量
    encoder.kinematic_inverse(linear_x, angular_z, target_left, target_right);
    if(linear_x == 0){
        motors.updateState(LOW);
    } else {
        motors.updateState(HIGH);
    }

    if(target_left >= 0) {
        pid_left.SetControllerDirection(QuickPID::Action::direct);
    } else {
        pid_left.SetControllerDirection(QuickPID::Action::reverse);
    }

    if(target_right >= 0) {
        pid_right.SetControllerDirection(QuickPID::Action::direct);
    } else {
        pid_right.SetControllerDirection(QuickPID::Action::reverse);
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
    // 使用串口设置 micro-ROS 传输层。
    // set_microros_serial_transports(Serial); // 配置串口和波特率


    // 使用 micro_ros_string_utilities_set 函数设置到 odom_msg.header.frame_id 中
    odom_msg.header.frame_id = micro_ros_string_utilities_set(odom_msg.header.frame_id, "odom");
    odom_msg.child_frame_id = micro_ros_string_utilities_set(odom_msg.child_frame_id, "base_link");

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
        "/cmd_vel"
    );

    rclc_publisher_init_best_effort(
        &odom_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "odom"
    );
    
    // 设置 micro-ROS 执行器，并将订阅添加到其中。
    rclc_executor_init(&executor, &support.context, 3, &allocator);
    rclc_executor_add_subscription(&executor, &subscriber, &sub_msg, &twist_callback, ON_NEW_DATA);

    // 循环运行 micro-ROS 执行器以处理传入的消息。
    while (true){
        if (!rmw_uros_epoch_synchronized()){
            rmw_uros_sync_session(1000);
            // 如果时间同步成功，则将当前时间设置为MicroROS代理的时间，并输出调试信息。
            delay(10);
        }
        delay(100);
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
}

void setup()
{
    // 初始化串口
    Serial.begin(115200);

    //apply PID gains
    pid_left.SetTunings(kp, ki, kd);
    pid_right.SetTunings(kp, ki, kd);

    //turn the PID on
    pid_left.SetMode(QuickPID::Control::automatic);
    pid_right.SetMode(QuickPID::Control::automatic);

    // 在核心0上创建一个名为"microros_task"的任务，栈大小为10240
    xTaskCreatePinnedToCore(microros_task, "microros_task", 10240, NULL, 1, NULL, 0);
}

unsigned long previousMillis = 0; // 上一次打印的时间
unsigned long interval = 50;      // 间隔时间，单位为毫秒

void loop()
{
    delay(10);
    encoder.get_current_vel();
    pid_left.Compute();
    pid_right.Compute();
    if(target_left >= 0){
        motors.updateSpeed(1, left_pwm*100);
    } else {
        motors.updateSpeed(1, -left_pwm*100);
    }

    if(target_right >= 0){
        motors.updateSpeed(2, right_pwm*100);
    } else {
        motors.updateSpeed(2, -right_pwm*100);
    }
    
    unsigned long currentMillis = millis(); // 获取当前时间
    if (currentMillis - previousMillis >= interval) {                                 // 判断是否到达间隔时间
        previousMillis = currentMillis; // 记录上一次打印的时间
        // 用于获取当前的时间戳，并将其存储在消息的头部中
        int64_t stamp = rmw_uros_epoch_millis();
        // 获取机器人的位置和速度信息，并将其存储在一个ROS消息（odom_msg）中
        Encoder::Odom& odom = encoder.odom();
        odom_msg.header.stamp.sec = static_cast<int32_t>(stamp / 1000); // 秒部分
        odom_msg.header.stamp.nanosec = static_cast<uint32_t>((stamp % 1000) * 1e6); // 纳秒部分
        odom_msg.pose.pose.position.x = odom.x;
        odom_msg.pose.pose.position.y = odom.y;
        odom_msg.pose.pose.orientation.w = odom.quaternion.w;
        odom_msg.pose.pose.orientation.x = odom.quaternion.x;
        odom_msg.pose.pose.orientation.y = odom.quaternion.y;
        odom_msg.pose.pose.orientation.z = odom.quaternion.z;

        odom_msg.twist.twist.angular.z = odom.angular_speed;
        odom_msg.twist.twist.linear.x = odom.linear_speed;
        Serial.println(odom_msg.pose.pose.position.x,5);
        auto ret = rcl_publish(&odom_publisher, &odom_msg, NULL);
        Serial.println(ret);
    }
}
