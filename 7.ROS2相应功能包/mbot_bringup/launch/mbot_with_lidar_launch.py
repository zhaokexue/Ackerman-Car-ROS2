#!/usr/bin/env python3

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
# mbot_description
from launch.conditions import UnlessCondition
import xacro
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # mbot
    mbot_serial_port = LaunchConfiguration('mbot_serial_port', default='/dev/mbot')
    mbot_odom_topic = LaunchConfiguration('mbot_odom_topic', default='odom')
    mbot_odom_frame_id = LaunchConfiguration('mbot_odom_frame_id', default='odom')
    mbot_base_frame_id = LaunchConfiguration('mbot_base_frame_id', default='base_link')
    mbot_sub_twist = LaunchConfiguration('mbot_sub_twist', default='cmd_vel')
    mbot_diffCar_or_ackerCar = LaunchConfiguration('mbot_diffCar_or_ackerCar', default='false')

    # mbot_description
    share_dir = get_package_share_directory('mbot_description')
    xacro_file = os.path.join(share_dir, 'urdf', 'mbot.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()
    # rviz_config_file = os.path.join(share_dir, 'config', 'display.rviz')
    show_gui = LaunchConfiguration('show_gui', default='false')

    # lidar
    lidar_serial_port = LaunchConfiguration('lidar_serial_port', default='/dev/lidar')
    lidar_serial_baudrate = LaunchConfiguration('lidar_serial_baudrate', default='115200') #for A1/A2 is 115200
    lidar_frame_id = LaunchConfiguration('lidar_frame_id', default='lidar_link')
    lidar_inverted = LaunchConfiguration('lidar_inverted', default='false')
    lidar_angle_compensate = LaunchConfiguration('lidar_angle_compensate', default='true')

    return LaunchDescription([
        Node(
            package="mbot_bringup",
            executable="mbot_bringup",
            name="mbot_bringup",
            parameters=[{"serial_port": mbot_serial_port,
                         "odom_topic": mbot_odom_topic,
                         "odom_frame_id": mbot_odom_frame_id,
                         "base_frame_id": mbot_base_frame_id,
                         "sub_twist": mbot_sub_twist,
                         "diffCar_or_ackerCar": mbot_diffCar_or_ackerCar}],
            output="screen"),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_urdf}]),

        Node(
            condition=UnlessCondition(show_gui),
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher'),

        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{'serial_port': lidar_serial_port, 
                         'serial_baudrate': lidar_serial_baudrate, 
                         'frame_id': lidar_frame_id,
                         'inverted': lidar_inverted, 
                         'angle_compensate': lidar_angle_compensate}],
            output='screen'),

        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', rviz_config_file],
        #     output='screen')
    ])
