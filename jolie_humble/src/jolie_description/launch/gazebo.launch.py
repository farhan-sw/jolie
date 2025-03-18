import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration

def generate_launch_description():
    ros_distro = os.environ['ROS_DISTRO']
    # Jika ROS Humble, kita asumsikan menggunakan Ignition Gazebo.
    is_ignition = 'True' if ros_distro == 'humble' else 'False'
    
    nebula_description_path = Path(get_package_share_directory('jolie_description'))
    
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(get_package_share_directory('jolie_description'), 'urdf', 'nebula_4wd.urdf.xacro'),
        description="Path to the robot model file",
    )
    
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
    
    gazebo_resource_path = SetEnvironmentVariable(
        name='GAZEBO_SIM_RESOURCE_PATH',
        value=[
            str(Path(nebula_description_path).parent.resolve()),
        ]
    )
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
        launch_arguments=[
            ("gz_args", [" -v 4", " -r", " empty.sdf"])
        ]
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
    
    # Ubah mapping topik laser agar sesuai dengan namespace Ignition Gazebo
    # gz_ros2_bridge = Node(
    #     package='ros_gz_bridge',
    #     executable='parameter_bridge',
    #     arguments=[
    #         "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
    #         "/model/jolie/laser/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"
    #     ],
    #     output='screen'
    # )
    
    # lidar_bridge = Node(
    #     package='ros_gz_bridge',
    #     executable='parameter_bridge',
    #     name='lidar_bridge',
    #     output='screen',
    #     parameters=[{
    #         'use_sim_time': True
    #     }],
    #     arguments=[
    #         ['/link/rplidar_link/sensor/rplidar/scan' +
    #          '@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan']
    #     ],
    #     remappings=[
    #         (['/link/rplidar_link/sensor/rplidar/scan'],
    #          'scan')
    #     ])
    
    return LaunchDescription([
        model_arg,
        robot_state_publisher,
        gazebo_resource_path,
        gazebo,
        gz_spawn_entity,
        lidar_bridge,
    ])
