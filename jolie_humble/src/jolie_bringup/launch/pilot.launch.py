import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    
    joystick = IncludeLaunchDescription(
        launch_description_source=os.path.join(
            get_package_share_directory("sirius_controller"), 
            "launch", 
            "joystick_teleop.launch.py"
        )
    )
        
    return LaunchDescription([
        joystick,
    ])