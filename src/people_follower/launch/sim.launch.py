"""Gazebo + robot + RViz for the CS485 person-following project.

Picks the robot with robot:= and includes tiago.launch.py or turtlebot4.launch.py,
passing every other argument through. Both robots carry UWB anchors 1-4 and
expose the same interface:
drive with geometry_msgs/TwistStamped on /cmd_vel, odometry on /odom, the
2D laser on /scan, and the same SLAM / AMCL / Nav2 options.

    ros2 launch people_follower sim.launch.py                         # TIAGo in box_room
    ros2 launch people_follower sim.launch.py robot:=turtlebot4       # TurtleBot 4 in box_room
    ros2 launch people_follower sim.launch.py world:=person_world     # open floor + one person
    ros2 launch people_follower sim.launch.py world:=simple_colored_warehouse   # UWB person
    ros2 launch people_follower sim.launch.py slam:=true              # build a map
    ros2 launch people_follower sim.launch.py localization:=true nav2:=true   # navigate on a map

With localization:=true the map defaults to <world>.yaml, looked up first in
people_follower/maps, then in turtlebot4_navigation/maps. Override with map:=/path/to/map.yaml.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

ROBOTS = ['tiago', 'turtlebot4']

ARGUMENTS = [
    DeclareLaunchArgument('robot', default_value='tiago', choices=ROBOTS,
                          description='Robot to simulate'),
    DeclareLaunchArgument('world', default_value='box_room',
                          description='World name (people_follower/worlds or a TB4 world)'),
    DeclareLaunchArgument('rviz', default_value='true',
                          choices=['true', 'false'], description='Start RViz'),
    DeclareLaunchArgument('model', default_value='standard',
                          choices=['standard', 'lite'],
                          description='TurtleBot 4 model (ignored for TIAGo)'),
    DeclareLaunchArgument('slam', default_value='false',
                          choices=['true', 'false'], description='Run SLAM to build a map'),
    DeclareLaunchArgument('localization', default_value='false',
                          choices=['true', 'false'], description='Run AMCL on a saved map'),
    DeclareLaunchArgument('nav2', default_value='false',
                          choices=['true', 'false'], description='Run Nav2'),
    DeclareLaunchArgument('map', default_value='',
                          description='Map yaml for localization (default: <world>.yaml)'),
]
for pose_element in ['x', 'y', 'z', 'yaw']:
    ARGUMENTS.append(DeclareLaunchArgument(pose_element, default_value='0.0',
                     description=f'{pose_element} component of the robot pose'))

FORWARDED = ['world', 'rviz', 'model', 'slam', 'localization', 'nav2', 'map', 'x', 'y', 'z', 'yaw']


def include_robot(context):
    robot = LaunchConfiguration('robot').perform(context)
    launch_file = os.path.join(get_package_share_directory('people_follower'), 'launch',
                               f'{robot}.launch.py')
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file),
        launch_arguments=[(name, LaunchConfiguration(name)) for name in FORWARDED])]


def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=include_robot))
    return ld
