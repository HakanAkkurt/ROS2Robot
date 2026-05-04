#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import LifecycleNode
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import GroupAction
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    use_slam  = LaunchConfiguration("use_slam")
    namespace = LaunchConfiguration("namespace")

    use_slam_arg = DeclareLaunchArgument("use_slam",    default_value="true")
    namespace_arg = DeclareLaunchArgument("namespace",  default_value="leader")

    bumperbot_description = get_package_share_directory("bumperbot_description")

    robot_description = ParameterValue(Command([
        "xacro ",
        os.path.join(bumperbot_description, "urdf", "rosmaster_a1.urdf.xacro"),
        " is_sim:=false",
        ]),
        value_type=str
    )

    configured_twist_mux = RewrittenYaml(
        source_file=os.path.join(
            get_package_share_directory('bumperbot_controller'),
            "config",
            "twist_mux_topics_a1.yaml"
        ),
        root_key=namespace,
        param_rewrites={},
        convert_types=True
    )

    robot_group = GroupAction(actions=[
        PushRosNamespace(namespace),

        Node(
            package='bumperbot_controller',
            executable='rosmaster_a1_node.py',
            name='rosmaster_a1_node',
            output='screen',
            remappings=[('cmd_vel', 'cmd_vel_final')]
        ),

        Node(
            package='bumperbot_controller',
            executable='rosmaster_a1_odometry.py',
            name='rosmaster_a1_odometry',
            output='screen',
            parameters=[{"namespace": namespace}]
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[
                {'use_sim_time': False},
                {"robot_description": robot_description},
                {'frame_prefix': PythonExpression(["'", namespace, "/'"])}
            ]
        ),

        Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_ros2_driver_node',
            namespace='/',
            output='screen',
            emulate_tty=True,
            parameters=[os.path.join(
                get_package_share_directory("bumperbot_bringup"),
                "config",
                "Tmini-Plus-SH.yaml"
            ), {
                "frame_id": PythonExpression(["'", namespace, "/laser_link'"]),
            }],
            remappings=[
                ('/scan', PythonExpression(["'/", namespace, "/scan'"])),
            ]
        ),

        IncludeLaunchDescription(
            os.path.join(
                get_package_share_directory("bumperbot_controller"),
                "launch",
                "rosmaster_a1_joystick_teleop.launch.py"
            ),
            launch_arguments={"use_sim_time": "false"}.items()
        ),

        Node(
            package='bumperbot_controller',
            executable='oled_system_monitor.py',
            name='oled_system_monitor',
            output='screen',
        ),

        IncludeLaunchDescription(
            os.path.join(
                get_package_share_directory("bumperbot_localization"),
                "launch",
                "global_localization_a1.launch.py"
            ),
            condition=UnlessCondition(use_slam),
            launch_arguments={
                "use_sim_time": "false",
                "namespace": namespace
            }.items()
        ),

        IncludeLaunchDescription(
            os.path.join(
                get_package_share_directory("bumperbot_mapping"),
                "launch",
                "slam.launch.py"
            ),
            condition=IfCondition(use_slam),
            launch_arguments={
                "use_sim_time": "false",
                "namespace": namespace
            }.items()
        ),

        Node(
            package="twist_mux",
            executable="twist_mux",
            name="twist_mux",
            output="screen",
            parameters=[
                    configured_twist_mux,
                    {"use_stamped": False}
            ],
            remappings=[('cmd_vel_out', 'cmd_vel_final')
            ]
        ),

        IncludeLaunchDescription(
            os.path.join(
                get_package_share_directory("bumperbot_navigation"),
                "launch",
                "navigation_a1.launch.py"
            ),
            launch_arguments={
                "use_sim_time": "false",
                "namespace": namespace
            }.items()
        ),

        Node(
            package='bumperbot_utils',
            executable='hailo_yolo_node.py',
            name='hailo_yolo_node',
            output='screen'
        ),
    ])

    return LaunchDescription([
        use_slam_arg,
        namespace_arg,
        robot_group
    ])