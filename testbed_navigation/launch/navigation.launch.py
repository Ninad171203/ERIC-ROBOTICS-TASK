import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    nav_pkg_share = get_package_share_directory('testbed_navigation')
    
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[os.path.join(nav_pkg_share, 'config', 'nav2_params.yaml')]
    )
    
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[os.path.join(nav_pkg_share, 'config', 'nav2_params.yaml')]
    )
    
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[os.path.join(nav_pkg_share, 'config', 'nav2_params.yaml')]
    )
    
    recoveries_server = Node(
        package='nav2_recoveries',
        executable='recoveries_server',
        name='recoveries_server',
        output='screen',
        parameters=[os.path.join(nav_pkg_share, 'config', 'nav2_params.yaml')]
    )
    
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'node_names': [
                'controller_server',
                'planner_server',
                'bt_navigator',
                'recoveries_server'
            ]},
            {'autostart': True}
        ]
    )
    
    return LaunchDescription([
        controller_server,
        planner_server,
        bt_navigator,
        recoveries_server,
        lifecycle_manager
    ])