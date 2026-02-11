import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch semubot eye controller node
        Node(
            package='semubot_eyes',
            executable='eye_controller', 
            name='eye_controller',
            output='screen',
        ),
        # Launch respeaker node
        Node(
            package='respeaker_ros',
            executable='respeaker_node',
            name='respeaker_node',
            output='screen',
        ),
    ])
