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
    xacro_path = os.path.join(share_dir, 'config', 'robot.urdf.xacro')
    rviz_config_file = os.path.join(share_dir, 'config', 'rviz2_offline.rviz')
    
    bag_file = LaunchConfiguration('bag_file')
    use_rviz = LaunchConfiguration('use_rviz')
    lidar_tf = LaunchConfiguration('lidar_tf')

    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            share_dir, 'config', 'params.yaml'),
        description='FPath to the ROS2 parameters file to use.')
        
    bag_flie_declare = DeclareLaunchArgument(
        'bag_file',
        default_value='',
        description='Bag file to run lio_sam offline.')
    
    use_rviz_declare = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Whether to start RViz or not (true/false).')
    
    lidar_tf_declare = DeclareLaunchArgument(
        'lidar_tf',
        default_value='false',
        description='Publish lidar to base_link transform.')

    print("urdf_file_name : {}".format(xacro_path))

    return LaunchDescription([
        params_declare,
        use_rviz_declare,
        lidar_tf_declare,
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments='0.0 0.0 0.0 0.0 0.0 0.0 map odom'.split(' '),
            parameters=[parameter_file],
            output='screen'
            ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments='0.0 0.0 0.0 0.0 0.0 0.0 base_link velodyne'.split(' '),
            parameters=[parameter_file],
            output='screen',
            condition=IfCondition(lidar_tf)
            ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro', ' ', xacro_path])
            }]
        ),
        Node(
            package='lio_sam',
            executable='lio_sam_offline_runner',
            arguments=[bag_file],
            parameters=[parameter_file],
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen',
            condition=IfCondition(use_rviz)
        )
    ])
