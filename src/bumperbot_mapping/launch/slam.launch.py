import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    # 1. Directories and Files
    mapping_pkg = get_package_share_directory("bumperbot_mapping")
    
    # 2. Launch Configurations
    use_sim_time = LaunchConfiguration("use_sim_time")
    namespace = LaunchConfiguration("namespace")
    
    # 3. Dynamic Frame Substitutions for SLAM
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'odom_frame': PythonExpression(["'", namespace, "/odom'"]),
        'map_frame': PythonExpression(["'", namespace, "/map'"]),
        'base_frame': PythonExpression(["'", namespace, "/base_footprint'"]),
        'scan_topic': PythonExpression(["'/", namespace, "/scan'"])
    }

    # 4. Rewrite SLAM YAML
    configured_slam_params = RewrittenYaml(
        source_file=os.path.join(mapping_pkg, "config", "slam_toolbox.yaml"),
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True
    )

    # 5. Nodes
    nav2_map_saver = Node(
        package="nav2_map_server",
        executable="map_saver_server",
        name="map_saver_server",
        output="screen",
        parameters=[{
            "save_map_timeout": 5.0,
            "use_sim_time": use_sim_time,
        }],
    )

    slam_toolbox = Node(
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[configured_slam_params],
        remappings=[
            ('/map',          PythonExpression(["'/", namespace, "/map'"])),
            ('/map_metadata', PythonExpression(["'/", namespace, "/map_metadata'"])),
            ('/tf',           '/tf'),
            ('/tf_static',    '/tf_static'),
        ]
    )

    # Note: Lifecycle nodes check for ROS Distro (Humble logic)
    ros_distro = os.environ.get("ROS_DISTRO", "humble")
    lifecycle_nodes = ["map_saver_server"]
    if ros_distro != "humble":
        lifecycle_nodes.append("slam_toolbox")

    nav2_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_slam",
        output="screen",
        parameters=[
            {"node_names": lifecycle_nodes},
            {"use_sim_time": use_sim_time},
            {"autostart": True}
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("namespace", default_value="leader"),
        nav2_map_saver,
        slam_toolbox,
        nav2_lifecycle_manager
    ])