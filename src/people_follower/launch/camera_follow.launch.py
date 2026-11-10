"""Follow a person with the camera: YOLO detection + depth + tracking + person_follower.

The follower's commands go through safety_guard (/cmd_vel_unsafe -> /cmd_vel), which
stops the robot short of anything: people, walls, low objects such as the dock.

Run alongside sim.launch.py, with the robot undocked (scripts/launch_camera_follow.sh
undocks and then runs this).

    ros2 launch people_follower camera_follow.launch.py
    ros2 launch people_follower camera_follow.launch.py noise:=true      # real-world errors
    ros2 launch people_follower camera_follow.launch.py follow_distance:=1.5
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def guard(context):
    noise = LaunchConfiguration('noise').perform(context) == 'true'
    return [Node(
        package='people_follower', executable='safety_guard', output='screen',
        parameters=[{
            'use_sim_time': True,
            'depth_topic': ('/oakd_noisy/rgb/preview/depth' if noise
                            else '/oakd/rgb/preview/depth'),
        }])]


def generate_launch_description():
    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('people_follower'), 'launch',
            'person_detector.launch.py')),
        launch_arguments=[('noise', LaunchConfiguration('noise'))])
    follower = Node(
        package='people_follower', executable='person_follower', output='screen',
        parameters=[{
            'use_sim_time': True,
            'cmd_topic': '/cmd_vel_unsafe',
            'follow_distance': ParameterValue(LaunchConfiguration('follow_distance'),
                                              value_type=float),
        }])
    return LaunchDescription([
        DeclareLaunchArgument('noise', default_value='false', choices=['true', 'false'],
                              description='Add real-world camera and detection errors'),
        DeclareLaunchArgument('follow_distance', default_value='1.0',
                              description="Metres to stay behind the person's body centre"),
        perception,
        follower,
        OpaqueFunction(function=guard),
    ])
