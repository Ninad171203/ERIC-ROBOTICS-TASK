import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true"
    )
    
    nav2_config_arg = DeclareLaunchArgument(
        "nav2_config",
        default_value=os.path.join(
            get_package_share_directory("testbed_navigation"),
            "config",
            "nav2_params.yaml"
        )
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    nav2_config = LaunchConfiguration("nav2_config")

    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[nav2_config]
    )

    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[nav2_config]
    )

    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[nav2_config]
    )

    recoveries_server = Node(
        package="nav2_recoveries",
        executable="recoveries_server",
        name="recoveries_server",
        output="screen",
        parameters=[nav2_config]
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            {"node_names": [
                "controller_server",
                "planner_server",
                "bt_navigator",
                "recoveries_server"
            ]},
            {"use_sim_time": use_sim_time},
            {"autostart": True}
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        nav2_config_arg,
        controller_server,
        planner_server,
        bt_navigator,
        recoveries_server,
        lifecycle_manager
    ])