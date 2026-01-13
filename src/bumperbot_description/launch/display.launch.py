import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PythonExpression

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bumperbot_description = get_package_share_directory("bumperbot_description")

    robot_name_arg = DeclareLaunchArgument(
        "robot_name", 
        default_value="rosmaster_a1",
        description="Choose robot model: 'bumperbot' or 'rosmaster_a1'"
    )

    model_path_expr = PythonExpression([
        "'", os.path.join(bumperbot_description, "urdf", ""), "' + '", 
        LaunchConfiguration("robot_name"), ".urdf.xacro'"
    ])

    model_arg = DeclareLaunchArgument(
        name="model", 
        default_value=model_path_expr,
        description="Absolute path to robot urdf file"
    )

    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]),
                                       value_type=str)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(bumperbot_description, "rviz", "display.rviz")],
    )

    return LaunchDescription([
        robot_name_arg,
        model_arg,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node
    ])