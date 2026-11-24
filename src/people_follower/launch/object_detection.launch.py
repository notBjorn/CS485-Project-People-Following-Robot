
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    package_share = Path(
        get_package_share_directory('people_follower')
    )

    model_path = (
        package_share
        / 'models'
        / 'homeobjects_pilot_best.pt'
    )

    return LaunchDescription([
        Node(
            package='people_follower',
            executable='object_detection',
            name='object_detection_node',
            output='screen',
            parameters=[{
                'model_path': str(model_path),
                'confidence': 0.5,
                'image_topic': '/oakd/rgb/preview/image_raw'
            }]
        )
    ])

