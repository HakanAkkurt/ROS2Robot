import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import LifecycleNode
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_slam = LaunchConfiguration("use_slam")

    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="true"
    )

    use_display_arg = DeclareLaunchArgument(
        'use_display',
        default_value='false',
        description='Whether to display the YOLO OpenCV window'
    )

    # Rosmaster Node
    hardware_interface = Node(
        package='bumperbot_controller',
        executable='rosmaster_a1_node.py',
        name='rosmaster_a1_node',
        output='screen',
        remappings=[('/cmd_vel', '/cmd_vel_final')]
    )

    odometry_node = Node(
        package='bumperbot_controller',
        executable='rosmaster_a1_odometry.py',
        name='rosmaster_a1_odometry',
        output='screen',
    )

    laser_driver = LifecycleNode(
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
            )],
    )

    bumperbot_description = get_package_share_directory("bumperbot_description")

    robot_description = ParameterValue(Command([
        "xacro ",
        os.path.join(bumperbot_description, "urdf", "rosmaster_a1.urdf.xacro"),
        " is_sim:=true", # Hardware interface not needed
        ]),
        value_type=str
    )
    
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {'use_sim_time': False},
            {"robot_description": robot_description}
        ]
    )

    tf2_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_laser',
        output='screen',
        arguments=['0', '0', '0.02', '0', '0', '0', '1', 'base_link', 'laser_frame']
    )
    
    joystick = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_controller"),
            "launch",
            "rosmaster_a1_joystick_teleop.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "False"
        }.items()
    )

    oled_display = Node(
        package='bumperbot_controller',
        executable='oled_system_monitor.py',
        name='oled_system_monitor',
        output='screen',
    )

    localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_localization"),
            "launch",
            "global_localization_a1.launch.py"
        ),
        condition=UnlessCondition(use_slam)
    )

    slam = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_mapping"),
            "launch",
            "slam.launch.py"
        ),
        condition=IfCondition(use_slam),
        launch_arguments={
            "use_sim_time": "false"
        }.items()
    )

    nav2_twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        output="screen",
        parameters=[
            os.path.join(
                get_package_share_directory('bumperbot_controller'),
                "config",
                "twist_mux_topics_a1.yaml")
        ],
        remappings=[('/cmd_vel_out', '/cmd_vel_final')]
    )

    navigation = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_navigation"),
            "launch",
            "navigation_a1.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "false"
        }.items()
    )
    
    yolo_node = Node(
        package='bumperbot_utils',
        executable='yolo_navigation_monitor.py',
        name='yolo_navigation_monitor',
        parameters=[{
            'use_display': LaunchConfiguration('use_display')
        }],
        output='screen',
    )

    return LaunchDescription([
        use_slam_arg,
        use_display_arg,
        hardware_interface,
        odometry_node,
        laser_driver,
        robot_state_publisher_node,
        tf2_node,
        joystick,
        oled_display,
        localization,
        slam,
        nav2_twist_mux,
        navigation,
        yolo_node
    ])