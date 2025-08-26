# Launch via:
# ros2 launch speed_controlled_ros2_ur5_interface estop_controller.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    """Launch E-stop individually."""

    # Path to ros2_ur5_interface package
    ros2_ur5_interface_dir = get_package_share_directory('speed_controlled_ros2_ur5_interface')

    estop_controller_node = Node(
        package='speed_controlled_ros2_ur5_interface',
        executable='estop_controller_node',
        name='estop_controller_node',
        output='screen'
    )

    # Create a group action for the launch description
    launch_group = GroupAction(
        actions=[
            estop_controller_node
        ]
    )

    # Return the LaunchDescription
    return LaunchDescription([
        launch_group
    ])