import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression

def generate_launch_description():

    controller_type_arg = DeclareLaunchArgument(
        'controller_type',
        default_value='simple',
        description='Type of controller to use: simple or omniwheel'
    )

    controller_type = LaunchConfiguration('controller_type')

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "controller_manager",
        ],
    )

    simple_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["simple_velocity_controller",
                "--controller-manager",
                "controller_manager"
        ],
        condition=IfCondition(PythonExpression(["'", controller_type, "' == 'simple'"]))
    )
    
    omniwheel_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "omni_wheel_controller",
            "--controller-manager",
            "controller_manager",
        ],
        condition=IfCondition(PythonExpression(["'", controller_type, "' == 'omniwheel'"]))
    )
    
    return LaunchDescription(
        [
            controller_type_arg,
            joint_state_broadcaster_spawner,
            simple_controller,
            omniwheel_controller
        ]
    )