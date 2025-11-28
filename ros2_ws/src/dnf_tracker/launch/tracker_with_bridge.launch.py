from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='coppelia_bridge',
            executable='bridge_node',
            name='coppelia_bridge'
        ),
        Node(
            package='dnf_tracker',
            executable='cube_tracker',
            name='cube_tracker'
        )
    ])
