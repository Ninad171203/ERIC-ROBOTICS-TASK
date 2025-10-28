import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    map_loader_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('testbed_navigation'),
                'launch',
                'map_loader.launch.py'
            )
        ])
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('testbed_navigation'),
                'launch',
                'localization.launch.py'
            )
        ])
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('testbed_navigation'),
                'launch',
                'navigation.launch.py'
            )
        ])
    )

    return LaunchDescription([
        map_loader_launch,
        localization_launch,
        navigation_launch
    ])