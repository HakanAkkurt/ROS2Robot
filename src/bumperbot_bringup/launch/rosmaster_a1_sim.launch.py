#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_slam = LaunchConfiguration("use_slam")
    namespace = LaunchConfiguration("namespace")

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

    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="leader",
        description="Namespace for the robot"
    )

    # 3. Get Package Directories
    pkg_desc = get_package_share_directory("bumperbot_description")
    pkg_controller = get_package_share_directory("bumperbot_controller")
    pkg_localization = get_package_share_directory("bumperbot_localization")
    pkg_mapping = get_package_share_directory("bumperbot_mapping")
    pkg_navigation = get_package_share_directory("bumperbot_navigation")
    pkg_utils = get_package_share_directory("bumperbot_utils")

    launch_params = {
        "use_sim_time": use_sim_time
    }
    
    # This GroupAction applies the namespace to everything inside
    robot_group = GroupAction(
        actions=[
            PushRosNamespace(namespace),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(pkg_desc, "launch", "gazebo.launch.py")
                ]),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "robot_name": "rosmaster_a1"
                }.items(),
            ),
            
            # Robot Controller
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(pkg_controller, "launch", "controller_rosmaster_a1_sim.launch.py")
                ]),
                launch_arguments=launch_params.items()
            ),

            # Odometry Node
            Node(
                package="bumperbot_controller",
                executable="ackermann_odometry_sim.py",
                name="ackermann_odometry_sim",
                output="screen",
                parameters=[launch_params]
            ),
            
            # Localization (AMCL) - active if SLAM is false
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    pkg_localization, "launch", "global_localization_a1.launch.py"
                )),
                condition=UnlessCondition(use_slam),
                launch_arguments=launch_params.items()
            ),

            # SLAM - active if SLAM is true
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    pkg_mapping, "launch", "slam.launch.py"
                )),
                condition=IfCondition(use_slam),
                launch_arguments=launch_params.items()
            ),

            # Navigation Stack (Nav2)
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    pkg_navigation, "launch", "navigation_a1.launch.py"
                )),
                launch_arguments=launch_params.items()
            ),

            # RViz Visualization
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", os.path.join(
                        pkg_navigation, "rviz", "nav2_leader_view.rviz"
                    )
                ],
                output="screen",
                parameters=[launch_params]
            ),

            # YOLO Detection
            # Node(
            #     package='bumperbot_utils',
            #     executable='yolo_detection_monitor.py',
            #     name='yolo_detection_monitor',
            #     parameters=[{
            #         'use_display': LaunchConfiguration('use_display'),
            #         'use_sim_time': use_sim_time
            #     }],
            #     output='screen'
            # )
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        use_slam_arg,
        use_display_arg,
        namespace_arg,
        robot_group
    ])