"""Walk the person in person_world through waypoints (run alongside sim.launch.py).

Starts the ROS <-> Gazebo bridges for the person's cmd_vel and odometry, plus
the human_waypoints node.

    ros2 launch people_follower human_waypoints.launch.py
    ros2 launch people_follower human_waypoints.launch.py params_file:=/path/to/waypoints.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('people_follower'), 'config', 'human_waypoints.yaml')

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='human_bridge',
        output='screen',
        arguments=['/model/human/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                   '/model/human/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry'])

    waypoints = Node(
        package='people_follower', executable='human_waypoints', output='screen',
        parameters=[LaunchConfiguration('params_file'), {'use_sim_time': True}])

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='YAML with human_waypoints parameters'),
        bridge,
        waypoints,
    ])
