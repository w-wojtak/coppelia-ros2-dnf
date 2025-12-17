# file: dnf_system/launch/dnf_system_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Cube tracker node
        Node(
            package='dnf_system',
            executable='cube_input_node',
            name='cube_input',
            output='screen'
        ),

        # DNF learning node
        Node(
            package='dnf_system',
            executable='dnf_learning_node',
            name='dnf_learning_node',
            output='screen'
        ),

        # CoppeliaSim bridge node
        Node(
            package='coppelia_bridge',
            executable='bridge_node',
            name='coppelia_bridge'
        )
    ])
