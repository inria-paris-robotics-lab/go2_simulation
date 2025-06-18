from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments for parameters
    simulator_arg = DeclareLaunchArgument(
        'simulator',
        default_value='pybullet',
        description="Which simulator to use 'pybullet' or 'simple'"
    )

    simulator = LaunchConfiguration('simulator')

    # Node configuration
    go2_simulation_node = Node(
        package='go2_simulation',  # Replace with the actual package name
        executable='simulation_node',  # Replace with the actual node executable name
        name='simulation_node',
        output='screen',
        parameters=[
            {
                'simulator': simulator,
            }
        ]
    )

    go2_viewer_node = Node(
        package='go2_simulation',
        executable='viewer_node',
        name='viewer_node',
        output='screen',
        condition=IfCondition(PythonExpression(["'", simulator, "' == 'simple'"]))
    )

    # Launch description
    return LaunchDescription([
        simulator_arg,
        go2_simulation_node,
        go2_viewer_node
    ])
