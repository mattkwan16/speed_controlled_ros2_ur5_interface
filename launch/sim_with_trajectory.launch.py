import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    """Launch the UR5 simulation and trajectory publisher."""

    # Path to ros2_ur5_interface package
    ros2_ur5_interface_dir = get_package_share_directory('speed_controlled_ros2_ur5_interface')
    sim_launch_file = os.path.join(ros2_ur5_interface_dir, 'launch', 'sim.launch.py')

    # Node: trajectory publisher
    trajectory_publisher_node = Node(
        package='speed_controlled_ros2_ur5_interface',
        executable='publish_trajectory_node',
        name='publish_trajectory_node',
        output='screen'
    )

    # Node: speed control
    speed_control_node = Node(
        package='speed_controlled_ros2_ur5_interface',
        executable='speed_control_node',
        name='speed_control_node',
        output='screen'
    )

    # Node: proximity sensor
    proximity_sensor_node = Node(
        package='speed_controlled_ros2_ur5_interface',
        executable='proximity_sensor_node',
        name='proximity_sensor_node',
        output='screen'
    )

    # Node: estop
    estop_controller_node = Node(
        package='speed_controlled_ros2_ur5_interface',
        executable='estop_controller_node',
        name='estop_controller_node',
        output='screen'
    )

    # Create a group action for the launch description
    launch_group = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(sim_launch_file)
            ),
            trajectory_publisher_node,
            speed_control_node,
            proximity_sensor_node,
            estop_controller_node
        ]
    )

    # Return the LaunchDescription
    return LaunchDescription([
        launch_group
    ])
