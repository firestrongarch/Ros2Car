import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import TimerAction

def generate_launch_description():
    #=============================1.定位到包的地址=============================================================
    ros2car_dir = get_package_share_directory('ros2car')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    
    #=============================2.声明参数，获取配置文件路径===================================================
    # use_sim_time 这里要设置成true,因为gazebo是仿真环境，其时间是通过/clock话题获取，而不是系统时间
    use_sim_time = LaunchConfiguration('use_sim_time', default='true') 
    map_yaml_path = LaunchConfiguration('map',default=os.path.join(ros2car_dir,'maps','map.yaml'))
    nav2_param_path = LaunchConfiguration('params_file',default=os.path.join(ros2car_dir,'config','nav2.yaml'))
    rviz_config_dir = os.path.join(ros2car_dir,'rviz','ros2car.rviz')
    rviz_config_dir2 = os.path.join(nav2_bringup_dir,'rviz','nav2_default_view.rviz')


    sdf_file = os.path.join(ros2car_dir, 'models', 'car', 'model.sdf')

    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir2],
        # parameters=[{'use_sim_time': use_sim_time}],
        output='screen')


    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc},
        ]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
    )

    car_bringup_node = Node(
        package='ros2car',
        executable='car_bringup',
        name='car_bringup',
        output='screen',
    )

    # 添加静态变换发布器节点
    static_tf_odom_to_laserframe = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'laser_frame'],
        output='screen',
    )

    static_tf_odom_to_car = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'car'],
        output='screen',
    )

    static_tf_odom_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        output='screen',
    )

    #=============================3.声明启动launch文件，传入：地图路径、是否使用仿真时间以及nav2参数文件==============
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_bringup_dir,'/launch','/bringup_launch.py']),
        launch_arguments={
            'map': map_yaml_path,
            'use_sim_time': use_sim_time,
            'params_file': nav2_param_path,
        }.items(),
    )

    return LaunchDescription([
        rviz_node,
        robot_state_publisher,
        car_bringup_node,
        joint_state_publisher,
        static_tf_odom_to_laserframe,
        static_tf_odom_to_car,
        static_tf_odom_to_base,
        TimerAction(
            period=2.0,  # 这里设置延迟时间为2秒
            actions=[nav2_bringup_launch]
        ),
    ])
