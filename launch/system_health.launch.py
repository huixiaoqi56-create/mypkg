from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mypkg',
            executable='system_health_monitor',
            name='system_health_monitor',
            output='screen'
        ),
        Node(
            package='mypkg',
            executable='system_health_listener',
            name='system_health_listener',
            output='screen'
        ),
    ])
