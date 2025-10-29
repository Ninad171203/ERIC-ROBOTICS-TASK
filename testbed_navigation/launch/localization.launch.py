import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    nav_pkg_share = get_package_share_directory('testbed_navigation')
    
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[os.path.join(nav_pkg_share, 'config', 'amcl_params.yaml')]
    )
    
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'node_names': ['amcl']},
            {'autostart': True}
        ]
    )
    
    return LaunchDescription([
        amcl_node,
        lifecycle_manager
    ])