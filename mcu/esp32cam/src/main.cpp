#include <Arduino.h>
#include <cstdint>
#include <micro_ros_platformio.h>
#include <WiFi.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <sensor_msgs/msg/compressed_image.h>
#include <micro_ros_utilities/string_utilities.h>
#include <micro_ros_utilities/type_utilities.h>

#define CAMERA_MODEL_AI_THINKER // Has PSRAM
#include "camera_pins.h"

#include "OV2640.h"
OV2640 cam;

// 定义 ROS2 执行器和支持结构
rclc_executor_t executor;
rclc_support_t support;
// 定义 ROS2 内存分配器
rcl_allocator_t allocator;
// 定义 ROS2 节点和发布者
rcl_node_t node;

rcl_timer_t timer;
rcl_publisher_t img_publisher;
sensor_msgs__msg__CompressedImage img_msg;

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

    // 使用 micro_ros_string_utilities_set 函数设置到 odom_msg.header.frame_id 中
    img_msg.header.frame_id = micro_ros_string_utilities_set(img_msg.header.frame_id, "camera");
    img_msg.format = micro_ros_string_utilities_set(img_msg.format, "jpeg");

    // 等待 2 秒，以便网络连接得到建立。
    delay(2000);

    // 设置 micro-ROS 支持结构、节点和订阅。
    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "camera", "", &support);
    rclc_publisher_init_default(
        &img_publisher, 
        &node, 
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, CompressedImage), 
        "camera/compressed"
    );

    // 设置 micro-ROS 执行器
    rclc_executor_init(&executor, &support.context, 1, &allocator);

    // 循环运行 micro-ROS 执行器以处理传入的消息。
    while (true){
        delay(100);
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
}

void setup()
{
    Serial.begin(115200);
    //while (!Serial);            //wait for serial connection.

    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;

    // Frame parameters
    //  config.frame_size = FRAMESIZE_UXGA;
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 12;
    config.fb_count = 2;

    #if defined(CAMERA_MODEL_ESP_EYE)
    pinMode(13, INPUT_PULLUP);
    pinMode(14, INPUT_PULLUP);
    #endif

    cam.init(config);

    // 在核心0上创建一个名为"microros_task"的任务，栈大小为10240
    xTaskCreatePinnedToCore(microros_task, "microros_task", 10240, NULL, 1, NULL, 0);
}

void loop()
{
    delay(100);
    // 用于获取当前的时间戳，并将其存储在消息的头部中
    int64_t stamp = rmw_uros_epoch_millis();
    img_msg.header.stamp.sec = static_cast<int32_t>(stamp / 1000); // 秒部分
    img_msg.header.stamp.nanosec = static_cast<uint32_t>((stamp % 1000) * 1e6); // 纳秒部分

    cam.run();
    img_msg.data.data = cam.getfb();
    img_msg.data.size = cam.getSize();

    auto ret = rcl_publish(&img_publisher, &img_msg, NULL);
}
