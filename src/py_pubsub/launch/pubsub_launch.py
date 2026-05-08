from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    config = os.path.join(
        os.path.dirname(__file__),
        '..',
        'config',
        'params.yaml'
    )
    return LaunchDescription([
        Node(
            package='py_pubsub',
            executable='publisher',
            name='minimal_publisher',
            output='screen',
            parameters=[config]
        ),
        Node(
            package='py_pubsub',
            executable='subscriber',
            name='minimal_subscriber',
            output='screen',
            parameters=[config]
        )
    ])