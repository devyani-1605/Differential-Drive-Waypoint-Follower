from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('diff_drive_robot')
    
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Waypoint follower node
    waypoint_follower = Node(
        package='diff_drive_robot',
        executable='waypoint_follower',
        name='waypoint_follower',
        parameters=[
            os.path.join(pkg_share, 'config', 'pid_params.yaml'),
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        waypoint_follower,
    ])