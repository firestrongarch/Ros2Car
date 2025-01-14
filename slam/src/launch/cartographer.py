
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():

    pkg_share = get_package_share_directory('ros2car')

    sdf_file = os.path.join(pkg_share, 'models', 'car', 'model.sdf')

    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()

    # 地图的分辨率
    resolution = LaunchConfiguration('resolution', default='0.05')
    # 地图的发布周期
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')
    # 配置文件夹路径
    configuration_directory = LaunchConfiguration('configuration_directory',default= os.path.join(pkg_share, 'config') )
    # 配置文件
    configuration_basename = LaunchConfiguration('configuration_basename', default='cartographer.lua')
    rviz_config_dir = os.path.join(pkg_share, 'rviz')+"/ros2car.rviz"
    print(f"rviz config in {rviz_config_dir}")

    #=====================声明节点，cartographer/occupancy_grid_node/rviz_node=================================
    
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        # parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-configuration_directory', configuration_directory,
                   '-configuration_basename', configuration_basename],
    )

    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        # parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
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
            {'frame_prefix': 'car/'},  # 确保没有前缀
        ],
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

    #===============================================定义启动文件========================================================
    return LaunchDescription([
        car_bringup_node,
        robot_state_publisher,
        joint_state_publisher,
        static_tf_odom_to_laserframe,
        static_tf_odom_to_car,
        static_tf_odom_to_base,
        rviz_node,
        cartographer_node,
        cartographer_occupancy_grid_node,
    ])
