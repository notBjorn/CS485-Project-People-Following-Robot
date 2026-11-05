"""Gazebo + TurtleBot 4 + RViz for the CS485 person-following project.

Mirrors turtlebot4_gz_bringup's turtlebot4_gz.launch.py, but starts Gazebo
here so this package's worlds/ folder can be added to GZ_SIM_RESOURCE_PATH
(TB4's sim.launch.py overwrites that variable). The world must be a plain
name, since the TB4 bridges build '/world/<world>/...' topics from it.

    ros2 launch people_follower sim.launch.py                    # box_room
    ros2 launch people_follower sim.launch.py world:=warehouse   # TB4 worlds: warehouse, maze, depot
    ros2 launch people_follower sim.launch.py slam:=true          # build a map
    ros2 launch people_follower sim.launch.py localization:=true nav2:=true   # navigate on a map

With localization:=true the map defaults to <world>.yaml, looked up first in
people_follower/maps, then in turtlebot4_navigation/maps. Override with map:=/path/to/map.yaml.
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument('world', default_value='box_room',
                          description='World name (people_follower/worlds or a TB4 world)'),
    DeclareLaunchArgument('rviz', default_value='true',
                          choices=['true', 'false'], description='Start RViz'),
    DeclareLaunchArgument('model', default_value='standard',
                          choices=['standard', 'lite'], description='TurtleBot 4 model'),
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


def find_map(context):
    """Resolve map:= to <world>.yaml when not given, so the spawn launch's localization uses it."""
    map_yaml = LaunchConfiguration('map').perform(context)
    if not map_yaml:
        world = LaunchConfiguration('world').perform(context)
        for pkg in ['people_follower', 'turtlebot4_navigation']:
            candidate = os.path.join(get_package_share_directory(pkg), 'maps', world + '.yaml')
            if os.path.isfile(candidate):
                map_yaml = candidate
                break
    if not map_yaml and LaunchConfiguration('localization').perform(context) == 'true':
        raise RuntimeError(
            'No map found for this world. Build one with slam:=true and scripts/save_map.sh, '
            'or pass map:=/path/to/map.yaml')

    robot_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('turtlebot4_gz_bringup'),
            'launch', 'turtlebot4_spawn.launch.py')),
        launch_arguments=[(name, LaunchConfiguration(name)) for name in
                          ['world', 'rviz', 'model', 'x', 'y', 'z', 'yaw',
                           'slam', 'localization', 'nav2']] + [('map', map_yaml)])
    return [robot_spawn]


def generate_launch_description():
    pkg_people_follower = get_package_share_directory('people_follower')
    pkg_tb4_gz_bringup = get_package_share_directory('turtlebot4_gz_bringup')
    pkg_tb4_description = get_package_share_directory('turtlebot4_description')
    pkg_create_description = get_package_share_directory('irobot_create_description')
    pkg_create_gz_bringup = get_package_share_directory('irobot_create_gz_bringup')

    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=':'.join([
            os.path.join(pkg_people_follower, 'worlds'),
            os.path.join(pkg_tb4_gz_bringup, 'worlds'),
            os.path.join(pkg_create_gz_bringup, 'worlds'),
            str(Path(pkg_tb4_description).parent.resolve()),
            str(Path(pkg_create_description).parent.resolve()),
        ]))

    gz_gui_plugin_path = SetEnvironmentVariable(
        name='GZ_GUI_PLUGIN_PATH',
        value=':'.join([
            os.path.join(get_package_share_directory('turtlebot4_gz_gui_plugins'), 'lib'),
            os.path.join(get_package_share_directory('irobot_create_gz_plugins'), 'lib'),
        ]))

    # The Create 3 model asks for the classic ogre engine, whose gpu_lidar reported only
    # its minimum range on hardware GPUs in our testing. Force ogre2 for the sensors.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments=[('gz_args', [
            LaunchConfiguration('world'), '.sdf', ' -r', ' -v 4',
            ' --render-engine-server ogre2', ' --gui-config ',
            PathJoinSubstitution([pkg_tb4_gz_bringup, 'gui',
                                  LaunchConfiguration('model'), 'gui.config']),
        ])])

    clock_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='clock_bridge',
        output='screen', arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'])

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(gz_resource_path)
    ld.add_action(gz_gui_plugin_path)
    ld.add_action(gazebo)
    ld.add_action(clock_bridge)
    ld.add_action(OpaqueFunction(function=find_map))
    return ld
