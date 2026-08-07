import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():

    share_dir = get_package_share_directory('lio_sam')
    parameter_file = LaunchConfiguration('params_file')

    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            share_dir, 'config', 'params.yaml'),
        description='FPath to the ROS2 parameters file to use.')

    return LaunchDescription([
        params_declare,
        
        Node(
            package='lio_sam',
            executable='lio_sam_mapViewerServer',
            parameters=[parameter_file],
            output='screen',
            emulate_tty=True
        )
    ])
