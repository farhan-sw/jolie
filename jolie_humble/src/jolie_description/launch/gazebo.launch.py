import os
from os import pathsep
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression

def generate_launch_description():
    jolie_description = get_package_share_directory('jolie_description')
    
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(jolie_description, 'urdf', 'nebula_4wd.urdf.xacro'),
        description="Path to the robot model file",
    )
    
    world_name_arg = DeclareLaunchArgument(
        name='world_name',
        default_value='empty',
        description='Name of the Gazebo world to load'
    )
    
    world_path = PathJoinSubstitution([
        jolie_description,  
        "worlds",
        PythonExpression(expression=["'", LaunchConfiguration('world_name'), "'", " + '.world'"])
    ])
    
    model_path = str(Path(jolie_description).parent.resolve())
    model_path += pathsep + os.path.join(jolie_description, 'models')
    
    gazebo_resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', model_path
    )
    
    ros_distro = os.environ['ROS_DISTRO']
    is_ignition = 'True' if ros_distro == 'humble' else 'False'
    
    
    robot_description = ParameterValue(Command([
        'xacro ',
        LaunchConfiguration('model'),
        ' is_ignition:=', is_ignition
        ]),
        value_type=str)
    
    # Robot state publisher (publishes /tf based on urdf and joint states)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
        launch_arguments={
                "gz_args": PythonExpression(["'", world_path, " -v 4 -r'"]),
        }.items()
        
    )
    
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            "-topic", "robot_description",
            "-name", "jolie",
            "-x", "0.0",  # Set the x position
            "-y", "0.0",  # Set the y position
            "-z", "0.1"   # Set the z position to a value above the ground
        ]
    )
    
    gz_ros2_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"
        ],
        output='screen'
    )

    
    return LaunchDescription([
        model_arg,
        world_name_arg,
        robot_state_publisher,
        gazebo_resource_path,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge,
    ])
