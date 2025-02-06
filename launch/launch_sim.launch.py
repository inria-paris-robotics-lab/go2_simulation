from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare launch arguments for parameters
    imu_topic_arg = DeclareLaunchArgument(
        'imu_topic_name',
        default_value='/utlidar/imu',
        description='Topic name for IMU data'
    )
    
    state_topic_arg = DeclareLaunchArgument(
        'state_topic_name',
        default_value='/lowstate',
        description='Topic name for state data'
    )
    
    cmd_topic_arg = DeclareLaunchArgument(
        'cmd_topic_name',
        default_value='/lowcmd',
        description='Topic name for command data'
    )
    # Get the path to the URDF file in the package
    package_share_directory = get_package_share_directory('go2_simulation')
    package_dir = package_share_directory.replace("install/go2_simulation/share", "src")
    urdf_path = os.path.join(package_dir, 'urdf', 'go2.urdf')

    robot_path_arg = DeclareLaunchArgument(
        'robot_path',
        default_value=urdf_path,
        description='Path to the robot URDF file'
    )

    # Node configuration
    go2_simulation_node = Node(
        package='go2_simulation',  # Replace with the actual package name
        executable='simulator_node',  # Replace with the actual node executable name
        name='simulator_node',
        parameters=[
            {
                'imu_topic_name': LaunchConfiguration('imu_topic_name'),
                'state_topic_name': LaunchConfiguration('state_topic_name'),
                'cmd_topic_name': LaunchConfiguration('cmd_topic_name'),
                'robot_path': LaunchConfiguration('robot_path')
            }
        ]
    )

    # Launch description
    return LaunchDescription([
        imu_topic_arg,
        state_topic_arg,
        cmd_topic_arg,
        robot_path_arg,
        go2_simulation_node
    ])
