import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='pipeline',
            default_value='ompl'
        ),
        launch.actions.DeclareLaunchArgument(
            name='db',
            default_value='false'
        ),
        launch.actions.DeclareLaunchArgument(
            name='db_path',
            default_value=get_package_share_directory(
                'learm_ros2_moveit_config') + '/default_warehouse_mongo_db'
        ),
        launch.actions.DeclareLaunchArgument(
            name='debug',
            default_value='false'
        ),
        launch.actions.DeclareLaunchArgument(
            name='load_robot_description',
            default_value='true'
        ),
        launch.actions.DeclareLaunchArgument(
            name='moveit_controller_manager',
            default_value='fake'
        ),
        launch.actions.DeclareLaunchArgument(
            name='fake_execution_type',
            default_value='interpolate'
        ),
        launch.actions.DeclareLaunchArgument(
            name='use_gui',
            default_value='false'
        ),
        launch.actions.DeclareLaunchArgument(
            name='use_rviz',
            default_value='true'
        ),
        launch_ros.actions.Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            # condition=launch.conditions.IfCondition(True),
            # condition=launch.conditions.IfCondition(
            #     "$(eval arg('moveit_controller_manager') == 'fake')")
        ),
        launch_ros.actions.Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher',
            # condition=launch.conditions.IfCondition(True),
            # condition=launch.conditions.IfCondition(
            #     "$(eval arg('moveit_controller_manager') == 'fake')")
        ),
        launch_ros.actions.Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            # condition=launch.conditions.IfCondition(True),
            # condition=launch.conditions.IfCondition(
            #     "$(eval arg('moveit_controller_manager') == 'fake')")
        ),
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='virtual_joint_broadcaster_0'
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
