import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

def generate_launch_description():

    nav_pkg = get_package_share_directory("bumperbot_navigation")
    
    bt_nav_to_pose = os.path.join(nav_pkg, 'behavior_tree', 'simple_navigation_w_replanning_and_recoveries_a1.xml')
    bt_nav_through_poses = os.path.join(nav_pkg, 'behavior_tree', 'simple_navigation.xml')

    use_sim_time = LaunchConfiguration("use_sim_time")
    namespace = LaunchConfiguration("namespace")

    # Using PythonExpression to dynamically join the namespace with the frame names at runtime
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'global_frame': PythonExpression(["'", namespace, "/map'"]),
        'robot_base_frame': PythonExpression(["'", namespace, "/base_footprint'"]),
        'odom_frame': PythonExpression(["'", namespace, "/odom'"]),
        'map_frame': PythonExpression(["'", namespace, "/map'"]),
        'base_frame': PythonExpression(["'", namespace, "/base_footprint'"]),
        'odom_topic': PythonExpression(["'/", namespace, "/odom'"]),
        'scan_topic': PythonExpression(["'/", namespace, "/scan'"]),
        'map_topic': PythonExpression(["'/", namespace, "/map'"])
    }

    # Helper function for RewrittenYaml
    def get_rewritten_yaml(file_name):
        return RewrittenYaml(
            source_file=os.path.join(nav_pkg, "config", file_name),
            root_key=namespace, # Dynamically sets the top-level YAML key
            param_rewrites=param_substitutions,
            convert_types=True
        )

    nav2_controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        output="screen",
        parameters=[get_rewritten_yaml("controller_server_a1.yaml")],
    )
    
    nav2_planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[get_rewritten_yaml("planner_server_a1.yaml")]
    )

    nav2_behaviors = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[get_rewritten_yaml("behavior_server_a1.yaml")],
    )
    
    nav2_bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[
            get_rewritten_yaml("bt_navigator_a1.yaml"),
            {"default_nav_to_pose_bt_xml": bt_nav_to_pose},
            {"default_nav_through_poses_bt_xml": bt_nav_through_poses}
        ],
    )

    nav2_smoother_server = Node(
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        output="screen",
        parameters=[get_rewritten_yaml("smoother_server.yaml")],
    )

    lifecycle_nodes = ["controller_server", "planner_server", "smoother_server", "bt_navigator", "behavior_server"]
    nav2_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
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
        nav2_controller_server,
        nav2_planner_server,
        nav2_smoother_server,
        nav2_behaviors,
        nav2_bt_navigator,
        nav2_lifecycle_manager
    ])