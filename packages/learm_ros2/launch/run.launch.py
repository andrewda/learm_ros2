import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    gui = LaunchConfiguration('gui', default='true')
    
    rviz_config = os.path.join(
        get_package_share_directory('learm_ros2_description'),
        'rviz/learm.rviz')

    urdf = os.path.join(
        get_package_share_directory('learm_ros2_description'),
        'urdf/learm.urdf')

    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            'gui',
            default_value='true'),
        Node(
            package='joint_state_publisher' + '_gui' if gui else '',
            executable='joint_state_publisher' + '_gui' if gui else '',
            name='joint_state_publisher',
            parameters=[{'robot_description': robot_desc}],
            arguments=[urdf]),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}],
            arguments=[urdf]),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            parameters=[{'robot_description': robot_desc}],
            arguments=['-d', rviz_config]),
    ])