import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory  # Pastikan ini diimpor


def generate_launch_description():
    # Get the package directories
    jolie_description = get_package_share_directory('jolie_description')
    
    # Get robot description from xacro file
    robot_description_raw = ParameterValue(
        Command([
            'xacro ', os.path.join(jolie_description, 'urdf', 'nebula_4wd.urdf.xacro'),
            ' is_simulation:=false',
            ' is_topicbased:=false'
        ]), 
        value_type=str
    )

    # Start robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_raw}],
    )
    
    # Controller Manager
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description_raw},
            {'use_sim_time': False},
            os.path.join(
                get_package_share_directory("sirius_controller"),
                "config",
                "sirius_controllers.yaml"
            )
        ]
    )
    
    return LaunchDescription([
        robot_state_publisher_node,
        controller_manager
    ])