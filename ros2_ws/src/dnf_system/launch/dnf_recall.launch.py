from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # CoppeliaSim bridge node (handles both cube tracking and robot commands)
        Node(
            package='coppelia_bridge',
            executable='bridge_node',
            name='coppelia_bridge',
            output='screen'
        ),

        # DNF recall node
        Node(
            package='dnf_system',
            executable='dnf_recall_node',
            name='dnf_recall_node',
            output='screen',
            parameters=[{
                'data_path': 'learned_data'
            }]
        ),
    ])