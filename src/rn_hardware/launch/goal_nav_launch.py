from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    bringup_dir = get_package_share_directory('rn_hardware')

    # Paths to your launch files
    localization_launch = os.path.join(bringup_dir, 'launch', 'localization_launch.py')
    nav2_launch = os.path.join(bringup_dir, 'launch', 'navigation_launch.py')

    return LaunchDescription([
        # Start localization first
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(localization_launch),
        ),

        # Start Nav2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
        ),

        # Start goal-to-goal script after 10 seconds
        TimerAction(
            period=10.0,
            actions=[
                Node(
                    package='rn_hardware',
                    executable='goal_to_goal_nav',
                    name='goal_navigator',
                    output='screen',
                )
            ]
        )
    ])