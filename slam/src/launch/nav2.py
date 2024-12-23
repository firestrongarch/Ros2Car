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

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'),
        ),
        launch_arguments={'gz_args': PathJoinSubstitution([
            ros2car_dir,
            'models',
            'world',
            'test.sdf',
        ]),
        # 'time':'--initial-sim-time 0'
        }.items(),
    )
    
    # Bridge to forward tf and joint states to ros2
    gz_topic = '/model/car'
    joint_state_gz_topic = '/world/demo' + gz_topic + '/joint_state'
    link_pose_gz_topic = gz_topic + '/pose'
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Clock (Gazebo -> ROS2)
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # Joint states (Gazebo -> ROS2)
            joint_state_gz_topic + '@sensor_msgs/msg/JointState[gz.msgs.Model',
            # Link poses (Gazebo -> ROS2)
            link_pose_gz_topic + '@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            link_pose_gz_topic + '_static@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            # Velocity and odometry (Gazebo -> ROS2)
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            # 雷达 (Gazebo -> ROS2)
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ],
        remappings=[
            (joint_state_gz_topic, 'joint_states'),
            (link_pose_gz_topic, '/tf'),
            (link_pose_gz_topic + '_static', '/tf_static'),
        ],
        parameters=[{
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
            # 'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir2],
        # parameters=[{'use_sim_time': use_sim_time}],
        output='screen')

    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'odom']
    )

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
        gazebo,
        bridge,
        robot_state_publisher,
        TimerAction(
            period=2.0,  # 这里设置延迟时间为2秒
            actions=[nav2_bringup_launch]
        ),
    ])
