
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='people_follower',
            executable='object_detection',
            name='object_detection_node',
            output='screen',
            parameters=[{
                'model_path': 'yolo26n.pt',
                'confidence': 0.5,
                'image_topic': '/oakd/rgb/preview/image_raw'
            }]
        )
    ])
