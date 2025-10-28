import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    waypoint_follower = Node(
        package="nav2_waypoint_follower",
        executable="waypoint_follower",
        name="waypoint_follower",
        output="screen",
        parameters=[{
            'use_sim_time': True
        }]
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_waypoint_follower",
        output="screen",
        parameters=[{
            'node_names': ['waypoint_follower'],
            'use_sim_time': True,
            'autostart': True
        }]
    )

    return LaunchDescription([
        waypoint_follower,
        lifecycle_manager
    ])