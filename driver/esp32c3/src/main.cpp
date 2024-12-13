#include <Arduino.h>
#include "drv8833.h"
#include <micro_ros_platformio.h>
#include <WiFi.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>

#include <PID_v1.h> 

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

// 回调函数，当接收到新的 Twist 消息时会被调用
void twist_callback(const void *msg_in)
{
  // 将接收到的消息指针转化为 geometry_msgs__msg__Twist 类型
  const geometry_msgs__msg__Twist *twist_msg = (const geometry_msgs__msg__Twist *)msg_in;
  // 从 Twist 消息中获取线速度和角速度
  float linear_x = twist_msg->linear.x;
  float angular_z = twist_msg->angular.z;
  // 打印接收到的速度信息
  Serial.printf("recv spped(%f,%f)\n", linear_x, angular_z);
  // 如果速度为零，则停止两个电机
  if (linear_x == 0 && angular_z == 0)
  {
    // motor.updateMotorSpeed(0, 0);
    // motor.updateMotorSpeed(1, 0);
    motors.updateSpeed(1,0);
    motors.updateSpeed(2,0);
    return;
  }

  // 根据线速度和角速度控制两个电机的转速
  if (linear_x > 0)
  {
    // motor.updateMotorSpeed(0, 70);
    // motor.updateMotorSpeed(1, 70);
    motors.updateSpeed(1,200);
    motors.updateSpeed(2,200);
  }

  // if (linear_x < 0)
  // {
  //   motor.updateMotorSpeed(0, -70);
  //   motor.updateMotorSpeed(1, -70);
  // }

  // if (angular_z > 0)
  // {
  //   motor.updateMotorSpeed(0, -70);
  //   motor.updateMotorSpeed(1, 70);
  // }

  // if (angular_z < 0)
  // {
  //   motor.updateMotorSpeed(0, 70);
  //   motor.updateMotorSpeed(1, -70);
  // }
}

// 这个函数是一个后台任务，负责设置和处理与 micro-ROS 代理的通信。
void microros_task(void *param)
{
  // 设置 micro-ROS 代理的 IP 地址。
  IPAddress agent_ip;
  agent_ip.fromString("192.168.1.104");

  // 使用 WiFi 网络和代理 IP 设置 micro-ROS 传输层。
  set_microros_wifi_transports("TP-LINK_520", "88888888", agent_ip, 8888);

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
  while (true)
  {
    delay(100);
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  }
}

void setup()
{
  // 初始化串口
  Serial.begin(115200);

  //将中断触发引脚（2号引脚）设置为INPUT_PULLUP（输入上拉）模式
  pinMode(button, INPUT_PULLUP); 
 
  //设置中断触发程序
  attachInterrupt(digitalPinToInterrupt(button), power, FALLING);

  // 在核心0上创建一个名为"microros_task"的任务，栈大小为10240
  xTaskCreatePinnedToCore(microros_task, "microros_task", 10240, NULL, 1, NULL, 0);
}

void loop()
{
  motors.updateState(state);
  delay(10);
}
