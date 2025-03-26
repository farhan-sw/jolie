import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    world_name_arg = DeclareLaunchArgument(
        name='world_name',
        default_value='empty',
        description='Name of the Gazebo world to load'
    )
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("jolie_description"), 
                "launch", 
                "gazebo.launch.py"
            )
        ),
        launch_arguments={
            "world_name": LaunchConfiguration("world_name")
        }.items()
    )
    
    controller = IncludeLaunchDescription(
        launch_description_source=os.path.join(
            get_package_share_directory("sirius_controller"), 
            "launch", 
            "controller.launch.py"
        ),
        launch_arguments={
            "controller_type": "omniwheel"
        }.items()
    )
    
    twist_mux = IncludeLaunchDescription(
        launch_description_source=os.path.join(
            get_package_share_directory("sirius_controller"), 
            "launch", 
            "twist_mux.launch.py"
        )
    )
    
    joystick = IncludeLaunchDescription(
        launch_description_source=os.path.join(
            get_package_share_directory("sirius_controller"), 
            "launch", 
            "joystick_teleop.launch.py"
        )
    )
        
    return LaunchDescription([
        world_name_arg,
        gazebo,
        controller,
        twist_mux,
        joystick,
    ])