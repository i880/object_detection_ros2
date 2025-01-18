from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='object_detection_ros2',
            executable='camera_node',
            name='camera_node',
            output='screen'
        ),
        Node(
            package='object_detection_ros2',
            executable='object_detection_node',
            name='object_detection_node',
            output='screen'
        ),
    ])
