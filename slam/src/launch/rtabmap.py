# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():

    pkg_ros_gz_sim_demos = get_package_share_directory('ros2car')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    sdf_file = os.path.join(pkg_ros_gz_sim_demos, 'models', 'car', 'model.sdf')

    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()

    rviz_launch_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Open RViz.'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'),
        ),
        launch_arguments={'gz_args': PathJoinSubstitution([
            pkg_ros_gz_sim_demos,
            'models',
            'world',
            'test.sdf'
        ])}.items(),
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
            gz_topic + '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            gz_topic + '/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            # # 单目相机 (Gazebo -> ROS2)
            # gz_topic + '/camera@sensor_msgs/msg/Image[gz.msgs.Image',
            # 深度相机 (Gazebo -> ROS2)
            gz_topic + '/sensor/rgbd/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            gz_topic + '/sensor/rgbd/image@sensor_msgs/msg/Image[gz.msgs.Image',
            gz_topic + '/sensor/rgbd/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        remappings=[
            (joint_state_gz_topic, 'joint_states'),
            (link_pose_gz_topic, '/tf'),
            (link_pose_gz_topic + '_static', '/tf_static'),
        ],
        parameters=[{'qos_overrides./tf_static.publisher.durability': 'transient_local'}],
        output='screen'
    )

    # Get the parser plugin convert sdf to urdf using robot_description topic
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

    # Launch rviz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_ros_gz_sim_demos, 'rviz', 'ros2car.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[
            {'use_sim_time': True},
        ]
    )

    rtabmap_parameters = [{
        'frame_id':'car/rgbd_frame',
        'subscribe_depth':True,
        'subscribe_odom_info':True,
        'approx_sync':False
    }]

    rtabmap_remappings=[
        # ('odom', '/model/car/odometry'), 
        ('rgb/image', '/model/car/sensor/rgbd/image'),
        ('rgb/camera_info', '/model/car/sensor/rgbd/camera_info'),
        ('depth/image', '/model/car/sensor/rgbd/depth_image')
    ]

    rtabmap_odom = Node(
        package='rtabmap_odom', executable='rgbd_odometry', output='screen',
        parameters= rtabmap_parameters,
        remappings= rtabmap_remappings
    )

    rtabmap_slam = Node(
        package='rtabmap_slam', executable='rtabmap', output='screen',
        parameters= rtabmap_parameters,
        remappings= rtabmap_remappings,
        arguments=['-d']
    )

    rtabmap_viz = Node(
        package='rtabmap_viz', executable='rtabmap_viz', output='screen',
        parameters = rtabmap_parameters,
        remappings = rtabmap_remappings
    )

    return LaunchDescription([
        rviz_launch_arg,
        gazebo,
        bridge,
        robot_state_publisher,
        rviz,
        rtabmap_odom,
        rtabmap_slam,
        rtabmap_viz
    ])
