"""Follow the person with UWB: uwb_localizer + uwb_follower (run alongside sim.launch.py).

The world needs the UWB plugin and a person wearing tag 1 (simple_colored_warehouse),
and the robot must be undocked. scripts/launch_follow.sh undocks and then runs this.

    ros2 launch people_follower uwb_follow.launch.py
    ros2 launch people_follower uwb_follow.launch.py follow_distance:=1.0
    ros2 launch people_follower uwb_follow.launch.py follow:=false     # only track the person
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    localizer = Node(
        package='people_follower', executable='uwb_localizer', output='screen',
        parameters=[{'use_sim_time': True}])
    follower = Node(
        package='people_follower', executable='uwb_follower', output='screen',
        parameters=[{
            'use_sim_time': True,
            'follow_distance': ParameterValue(LaunchConfiguration('follow_distance'),
                                              value_type=float),
        }],
        condition=IfCondition(LaunchConfiguration('follow')))
    return LaunchDescription([
        DeclareLaunchArgument('follow_distance', default_value='0.5',
                              description='Metres to stay behind the person, along their path'),
        DeclareLaunchArgument('follow', default_value='true', choices=['true', 'false'],
                              description='false: only run the localizer'),
        localizer,
        follower,
    ])
