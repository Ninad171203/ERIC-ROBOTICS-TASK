import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    bringup_pkg_share = get_package_share_directory('testbed_bringup')
    
    map_file = os.path.join(bringup_pkg_share, 'maps', 'testbed_world.yaml')
    
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_file}]
    )
    
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[
            {'node_names': ['map_server']},
            {'autostart': True}
        ]
    )
    
    return LaunchDescription([
        map_server_node,
        lifecycle_manager
    ])