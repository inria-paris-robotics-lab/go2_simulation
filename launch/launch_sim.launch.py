from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare launch arguments for parameters
    simulator_arg = DeclareLaunchArgument(
        'simulator',
        default_value='pybullet',
        description="Which simulator to use 'pybullet' or 'simple'"
    )

    # Node configuration
    go2_simulation_node = Node(
        package='go2_simulation',  # Replace with the actual package name
        executable='simulator_node',  # Replace with the actual node executable name
        name='simulator_node',
        parameters=[
            {
                'simulator': LaunchConfiguration('simulator'),
            }
        ]
    )

    # Launch description
    return LaunchDescription([
        simulator_arg,
        go2_simulation_node
    ])
