"""Walk a simulated person through waypoints (run alongside sim.launch.py).

Starts the ROS <-> Gazebo bridges for the person's cmd_vel and odometry, plus
the human_waypoints node.

    ros2 launch people_follower human_waypoints.launch.py
    ros2 launch people_follower human_waypoints.launch.py params_file:=/path/to/waypoints.yaml
    ros2 launch people_follower human_waypoints.launch.py person:=human2 params_file:=...

person is the person's Gazebo model name (default human). For another person the node
is named <person>_waypoints, so its parameter file must use that name as its top key.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_nodes(context):
    person = LaunchConfiguration('person').perform(context)
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name=f'{person}_bridge',
        output='screen',
        arguments=[f'/model/{person}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                   f'/model/{person}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry'])
    waypoints = Node(
        package='people_follower', executable='human_waypoints', output='screen',
        name='human_waypoints' if person == 'human' else f'{person}_waypoints',
        parameters=[LaunchConfiguration('params_file'),
                    {'use_sim_time': True, 'person': person}])
    return [bridge, waypoints]


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('people_follower'), 'config', 'human_waypoints.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='YAML with human_waypoints parameters'),
        DeclareLaunchArgument('person', default_value='human',
                              description="The person's Gazebo model name"),
        OpaqueFunction(function=launch_nodes),
    ])
