from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mypkg',
            executable='system_health_monitor',
            output='screen',
        ),
        Node(
            package='mypkg',
            executable='system_health_listener',
            parameters=[
                {'offline_timeout': 3.0},
                {'log_path': '/tmp/system_health_log.csv'},
            ],
            output='screen',
        ),
    ])
