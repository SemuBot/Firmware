# launch/semubot_core.launch.py


from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Generates the launch description for the core Semubot nodes."""
    return LaunchDescription([
        Node(
            package='audio_publisher_node',
            executable='audio_publisher_node', 
            name='audio_publisher_node',
            output='screen'
        ),
        Node(
            package='semubot_llm',
            executable='llm_controller',
            name='llm_controller',
            output='screen'
        ),
    ])
