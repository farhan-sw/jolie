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

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    controller_type = LaunchConfiguration('controller_type')
    use_sim_time = LaunchConfiguration('use_sim_time')

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "controller_manager",
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    simple_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["simple_velocity_controller",
                "--controller-manager",
                "controller_manager"
        ],
        condition=IfCondition(PythonExpression(["'", controller_type, "' == 'simple'"])),
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    omniwheel_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "omni_wheel_controller",
            "--controller-manager",
            "controller_manager",
        ],
        condition=IfCondition(PythonExpression(["'", controller_type, "' == 'omniwheel'"])),
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    return LaunchDescription(
        [
            controller_type_arg,
            use_sim_time_arg,
            joint_state_broadcaster_spawner,
            simple_controller,
            omniwheel_controller
        ]
    )