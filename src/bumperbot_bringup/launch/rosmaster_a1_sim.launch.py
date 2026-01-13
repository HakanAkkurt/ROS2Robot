#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_slam = LaunchConfiguration("use_slam")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true"
    )

    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="true"
    )

    use_display_arg = DeclareLaunchArgument(
        'use_display',
        default_value='true',
        description='Whether to display the YOLO OpenCV window'
    )

    pkg_desc = get_package_share_directory("bumperbot_description")
    pkg_controller = get_package_share_directory("bumperbot_controller")

    launch_params = {"use_sim_time": use_sim_time}
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_desc, "launch", "gazebo.launch.py")
        ]),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "robot_name": "rosmaster_a1"
        }.items(),
    )
    
    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_controller, "launch", "controller_rosmaster_a1_sim.launch.py")
        ]),
        launch_arguments=launch_params.items()
    )

    odometry = Node(
        package="bumperbot_controller",
        executable="ackermann_odometry_sim.py",
        name="ackermann_odometry_sim",
        output="screen",
        parameters=[launch_params]
    )
    
    localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_localization"),
            "launch",
            "global_localization_a1.launch.py"
        ),
        condition=UnlessCondition(use_slam),
        launch_arguments=launch_params.items()
    )

    slam = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_mapping"),
            "launch",
            "slam.launch.py"
        ),
        condition=IfCondition(use_slam),
        launch_arguments=launch_params.items()
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("bumperbot_navigation"),
            "launch", "navigation_a1.launch.py"
        )),
        launch_arguments=launch_params.items()
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(
                get_package_share_directory("bumperbot_navigation"),
                "rviz", "nav2_view.rviz"
            )
        ],
        output="screen",
        parameters=[launch_params]
    )

    yolo_node = Node(
        package='bumperbot_utils',
        executable='yolo_navigation_monitor.py',
        name='yolo_navigation_monitor',
        parameters=[{
            'use_display': LaunchConfiguration('use_display')
        }],
        output='screen'
    )

    return LaunchDescription([
        use_sim_time_arg,
        use_slam_arg,
        use_display_arg,
        gazebo,
        controller,
        odometry,
        localization,
        slam,
        navigation,
        rviz,
        # yolo_node
    ])