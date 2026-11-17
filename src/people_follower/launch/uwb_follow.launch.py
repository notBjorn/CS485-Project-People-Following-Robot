"""Follow the person with UWB: uwb_localizer + uwb_follower (run alongside sim.launch.py).

The world needs the UWB plugin and a person wearing tag 1 (simple_colored_warehouse).
The TurtleBot 4 must be undocked first; TIAGo has no dock. scripts/launch_follow.sh
detects the robot, undocks the TurtleBot 4 if needed, and then runs this.

    ros2 launch people_follower uwb_follow.launch.py                      # TIAGo
    ros2 launch people_follower uwb_follow.launch.py robot:=turtlebot4
    ros2 launch people_follower uwb_follow.launch.py follow_distance:=1.0
    ros2 launch people_follower uwb_follow.launch.py follow:=false     # only track the person

Per robot:
    localizer frame   TIAGo: base_footprint (on the floor, so the anchors' height in it
                      is their height above the floor, like tag_height_m);
                      TurtleBot 4: base_link
    follow_distance   TIAGo: 0.7 m (its base is 0.54 m across); TurtleBot 4: 0.5 m
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DEFAULTS = {
    'tiago': {'base_frame': 'base_footprint', 'follow_distance': 0.7},
    'turtlebot4': {'base_frame': 'base_link', 'follow_distance': 0.5},
}


def launch_nodes(context):
    robot = LaunchConfiguration('robot').perform(context)
    defaults = DEFAULTS[robot]
    follow_distance = LaunchConfiguration('follow_distance').perform(context)
    follow_distance = float(follow_distance) if follow_distance else defaults['follow_distance']

    localizer = Node(
        package='people_follower', executable='uwb_localizer', output='screen',
        parameters=[{'use_sim_time': True, 'base_frame': defaults['base_frame']}])
    nodes = [localizer]
    if LaunchConfiguration('follow').perform(context) == 'true':
        nodes.append(Node(
            package='people_follower', executable='uwb_follower', output='screen',
            parameters=[{'use_sim_time': True, 'follow_distance': follow_distance}]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='tiago', choices=list(DEFAULTS),
                              description='Robot being simulated'),
        DeclareLaunchArgument('follow_distance', default_value='',
                              description='Metres to stay behind the person, along their path '
                                          '(default: 0.7 for TIAGo, 0.5 for the TurtleBot 4)'),
        DeclareLaunchArgument('follow', default_value='true', choices=['true', 'false'],
                              description='false: only run the localizer'),
        OpaqueFunction(function=launch_nodes),
    ])
