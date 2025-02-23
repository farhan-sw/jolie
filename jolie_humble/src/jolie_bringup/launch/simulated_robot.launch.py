import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    gazebo = IncludeLaunchDescription(
        launch_description_source=os.path.join(
            get_package_share_directory("jolie_description"), 
            "launch", 
            "gazebo.launch.py"
        )
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
    
    joystick = IncludeLaunchDescription(
        launch_description_source=os.path.join(
            get_package_share_directory("sirius_controller"), 
            "launch", 
            "joystick_teleop.launch.py"
        )
    )
        
    return LaunchDescription([
        gazebo,
        controller,
        # joystick,
    ])