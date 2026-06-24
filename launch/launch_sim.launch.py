from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Declare launch arguments for parameters
    simulator_arg = DeclareLaunchArgument(
        "simulator", default_value="pybullet", description="Which simulator to use 'pybullet'"
    )

    robot_arg = DeclareLaunchArgument("robot", description="Which robot to simulate 'go2' or 'g1'")

    unlock_base_arg = DeclareLaunchArgument(
        "unlock_base",
        default_value="False",
        description="should the robot base be free from start, or should it simulate being hanged first",
    )

    # Actuated G1 DOF: 27 (waist_roll/pitch locked, mode 6) or 29 (mode 5). Selects
    # the URDF variant loaded by PyBullet. Ignored for go2. Default 27.
    dof_arg = DeclareLaunchArgument(
        "dof",
        default_value="27",
        description="Actuated G1 DOF: 27 (mode 6, waist roll/pitch locked) or 29 (mode 5). Ignored for go2.",
    )

    # Node configuration
    go2_simulation_node = Node(
        package="unitree_simulation",  # Replace with the actual package name
        executable="simulation_node",  # Replace with the actual node executable name
        name="simulation_node",
        output="screen",
        parameters=[
            {
                "simulator": LaunchConfiguration("simulator"),
                "robot": LaunchConfiguration("robot"),
                "unlock_base": LaunchConfiguration("unlock_base"),
                "dof": ParameterValue(LaunchConfiguration("dof"), value_type=int),
            }
        ],
    )

    # Launch description
    return LaunchDescription([simulator_arg, robot_arg, unlock_base_arg, dof_arg, go2_simulation_node])
