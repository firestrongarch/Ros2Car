# Ros2Car

## 1 硬件环境
### 1.1 驱动板
| 封装      | 数量 |
| ----------- | ----------- |
| EPS32C3-合宙      | 1       |
| DRV8833Q   | 1        |
| HDR-TH_2P-P2.54-V-F   | 1        |
| CONN-TH_XH2.54-6P   | 2        |
| lm2596模块   | 1        |

![这是图片](doc/schematic.svg "原理图")
### 1.2 上位机
    jetson nano开发板
### 1.3 电机参数
|    属性   | 参数 |
| ----------- | ----------- |
| 类型      | TT马达电机       |
| 重量   | 40g        |
| 减速比   | 1：48        |
| 工作电压   | 3-12V        |
| 编码器   | AB相增量式霍尔编码器        |
| 编码器线数   | 13线        |
| 空转   | 200RPM        |
| 额定扭矩   | 1.5kg.cm        |

### 1.4 底盘
![这是图片](doc/chassis.png "底盘")
## 2 上位机软件环境
| 软件      | 版本 |
| ----------- | ----------- |
| ubuntu      | 24.04       |
| ros2   | jazzy        |
| gazebo   | harmonic        |

```
# 安装ros2 
wget http://fishros.com/install -O fishros && . fishros
```

## 3 仿真运行
3.1 创建桥接器转换话题
```
# 桥接cmd_vel
ros2 run ros_gz_bridge parameter_bridge /cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist
# 桥接camera
ros2 run ros_gz_bridge parameter_bridge /camera@sensor_msgs/msg/Image@gz.msgs.Image
```
3.2 启动teleop控制小车移动
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# 或者话题映射到自定义话题
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=model/car/cmd_vel
```
3.3 启动luanch
```
source ./install/setup.zsh

# 自动配置桥接器和话题，启动gazebo和rviz，注意gazebo需要手动启动仿真
ros2 launch ros2car test.py
```
3.4 控制小车移动
![这是图片](doc/test.gif "底盘")

## 4 实物运行
### 4.1 串口驱动
```
# 下载ch341驱动和ch343驱动，在driver目录下运行
make
sudo make load
sudo make install

# 查看内核缓冲
sudo dmesg

# 查看usb
lsusb

# 设置串口权限，加入dialout用户组，username替换为你的用户名
sudo usermod -aG dialout username

# 查看用户组
cat /etc/group|grep 组名
```
### 4.2 配置platformIO
```
# 为clangd生成compile_commands.json
pio run -t compiledb
```
问题1：algorithm头文件无法找到，原因是riscv-esp32库默认使用C编译，不会生成C++头文件索引。但是用户代码使用C++编译，所以clangd会识别arduino中的c++部分，导致错误。 解决方法：使用 Suppress: [pp_file_not_found]字段解决

问题2: xtensa-esp32-elf-g++未知三元组。 解决方法：更新clangd服务器至19.2版本

### 4.3 手动配置micro_ros
(1)编译micro_ros_setup，该软件包基于ROS2，用于创建micro-ROS Agent和firmware
```
mkdir microros_ws
cd microros_ws
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

sudo apt update && rosdep update
rosdep install --from-path src --ignore-src -y

sudo apt-get install python3-pip

colcon build
source install/local_setup.zsh
```
(2)创建micro-ROS Agent，负责PC的ROS2和MCU上的micro-ROS之间的通信。
```
# Download micro-ROS Agent packages
ros2 run micro_ros_setup create_agent_ws.sh

# Build agent and source the workspace again
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash
```
(3) 编译MCU固件  参考链接[编译静态micro-ROS库](https://micro.ros.org/docs/tutorials/advanced/create_custom_static_library/)
```
# 适用于特定平台的 micro-ROS 独立模块
ros2 run micro_ros_setup component --help

# 创建esp32idf示例, 将创建一个固件文件夹，其中包含构建 micro-ROS 应用程序所需的代码
ros2 run micro_ros_setup create_firmware_ws.sh freertos esp32

# 配置 micro-ROS 固件
ros2 run micro_ros_setup configure_firmware.sh [configuration] [options]

# 构建固件
ros2 run micro_ros_setup build_firmware.sh

# 刷写固件
ros2 run micro_ros_setup flash_firmware.sh
---------

# 创建自定义静态 micro-ROS 库
ros2 run micro_ros_setup create_firmware_ws.sh generate_lib

# 下载完所有包后，需要创建几个文件，以便交叉编译自定义静态库和一组头文件：
touch my_custom_toolchain.cmake
touch my_custom_colcon.meta

# 配置cmake编译器路径, 以riscv为例,(path)替换为实际路径
set(CMAKE_C_COMPILER (path)/riscv32-esp-elf-gcc)
set(CMAKE_CXX_COMPILER (path)/riscv32-esp-elf-g++)
set(CMAKE_AR (path)/riscv32-esp-elf-gcc-ar)

# 编译自定义静态库
ros2 run micro_ros_setup build_firmware.sh $(pwd)/my_custom_toolchain.cmake $(pwd)/my_custom_colcon.meta
```

错误警告:bomb::bomb::bomb: : do not exist list index: 1 out of range (-1, 0)
```
# 错误原因是路径包含中文(史诗级巨坑),耗时两天才找到原因,很难想象2024年不支持UTF-8。仅将目录改为英文:
export LANG=en_US
xdg-user-dirs-gtk-update
export LANG=zh_CH
```

(4) 运行Agent
```
# docker直接运行,串口通信
sudo docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:$ROS_DISTRO serial --dev /dev/ttyUSB0 -v6

# UDPv4 micro-ROS Agent
docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:$ROS_DISTRO udp4 --port 8888 -v6

# Serial micro-ROS Agent
docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:$ROS_DISTRO serial --dev [YOUR BOARD PORT] -v6

# TCPv4 micro-ROS Agent
docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:$ROS_DISTRO tcp4 --port 8888 -v6

# CAN-FD micro-ROS Agent
docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:$ROS_DISTRO canfd --dev [YOUR CAN INTERFACE] -v6

```
### 4.3 开发Airm2m_core_esp32c3
(1)串口通信
```
// 初始化串口
Serial.begin(115200);
Serial.println(val)
```

### 4.4 开发esp32CAM
注意：ESP32-CAM 上的 IO0 和 GND 短接以进入下载模式（拔掉才可以运行程序！）。烧录时可能需要先复位或重新连接电脑。

