from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            parameters=[{"dev": "/dev/input/js0"}],
            output="screen"
        ),

        Node(
            package="bumperbot_controller",
            executable="rosmaster_a1_joystick_teleop.py",
            name="rosmaster_a1_joystick_teleop",
            output="screen"
        )
    ])