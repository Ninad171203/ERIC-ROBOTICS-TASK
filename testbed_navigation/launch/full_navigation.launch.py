import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    nav_pkg_share = get_package_share_directory('testbed_navigation')
    
    map_loader_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_pkg_share, 'launch', 'map_loader.launch.py')
        )
    )
    
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_pkg_share, 'launch', 'localization.launch.py')
        )
    )
    
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_pkg_share, 'launch', 'navigation.launch.py')
        )
    )
    
    return LaunchDescription([
        map_loader_launch,
        localization_launch,
        navigation_launch
    ])