#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_controller = get_package_share_directory("bumperbot_controller")

    joint_state_broadcaster = Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'joint_state_broadcaster',
                '--controller-manager', '/controller_manager'
            ],
            output='screen'
        )
        
    ackermann_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'ackermann_controller',
            '--controller-manager', '/controller_manager',
            '--param-file', os.path.join(pkg_controller, 'config', 'rosmaster_a1_controllers.yaml')
        ],
        output='screen'
    )
    
    
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'dev': '/dev/input/js0',
            'deadzone': 0.2,
            'autorepeat_rate': 20.0
        }],
        output='screen'
    )
    
    joystick_teleop = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_twist_joy",
        parameters=[os.path.join(pkg_controller, "config", "rosmaster_a1_teleop_joy.yaml")],
        output='screen'
    )
    
    twist2ackermann = Node(
        package='bumperbot_controller',
        executable='twist_to_ackermann.py',
        name='twist_to_ackermann',
        output='screen'
    )

    return LaunchDescription([
        joint_state_broadcaster,
        ackermann_controller,
        joy_node,
        joystick_teleop,
        twist2ackermann
    ])