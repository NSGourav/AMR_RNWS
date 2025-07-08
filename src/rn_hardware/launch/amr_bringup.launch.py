from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('rn_hardware')
    urdf_path = os.path.join(pkg_share, 'urdf', 'amr.urdf.xacro')
    controllers_path = os.path.join(pkg_share, 'config', 'amr_controllers.yaml')
    ekf_config_path = os.path.join(pkg_share, 'config', 'ekf.yaml')

    robot_description = Command(['xacro ', urdf_path])

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        ),

        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                {'robot_description': robot_description},
                controllers_path
            ],
            output='screen'
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            output='screen'
        ),
        Node(

            package ='rn_hardware',
            executable='odom.py',
            output='screen'
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
            output='screen'
        ),
        ##Node(
        ##    package='robot_localization',
        ##    executable='ekf_node',
        ##    name='ekf_filter_node',
        ##    output='screen',
        ##    parameters=[ekf_config_path]
       ## ),
        Node(
                package='rplidar_ros',
                executable='rplidar_composition',
                name='rplidar_node',
                output='screen',
                parameters=[{
                    'serial_port': '/dev/ttyUSB0',
                    'serial_baudrate': 115200,
                    'frame_id': 'rplidar_link',
                    'inverted': False,
                    'angle_compensate': True,
                }]
        )

    ])
