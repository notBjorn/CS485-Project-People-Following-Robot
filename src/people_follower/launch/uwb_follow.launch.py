"""Follow the person with UWB: uwb_localizer + uwb_follower (run alongside sim.launch.py).

The world needs the UWB plugin and a person wearing tag 1 (simple_colored_warehouse).
The TurtleBot 4 must be undocked first; TIAGo has no dock. scripts/launch_follow.sh
detects the robot, undocks the TurtleBot 4 if needed, and then runs this.

    ros2 launch people_follower uwb_follow.launch.py                      # TIAGo
    ros2 launch people_follower uwb_follow.launch.py robot:=turtlebot4
    ros2 launch people_follower uwb_follow.launch.py follow_distance:=1.0
    ros2 launch people_follower uwb_follow.launch.py follow:=false     # only track the person
    ros2 launch people_follower uwb_follow.launch.py max_linear:=0.5   # slower
    ros2 launch people_follower uwb_follow.launch.py start_enabled:=false
        # follower waits for /uwb_follower/enable or a /uwb_follower/follow_person goal

Per robot:
    localizer frame   TIAGo: base_footprint (on the floor, so the anchors' height in it
                      is their height above the floor, like tag_height_m);
                      TurtleBot 4: base_link
    robot_radius_m    TIAGo: 0.27 m (its base is 0.50 m across); TurtleBot 4: 0.18 m
    max_linear        TIAGo: 1.0 m/s (its base's top speed); TurtleBot 4: 0.3 m/s
    follow_distance   robot_radius_m + 2 ft (0.61 m): TIAGo 0.88 m, TurtleBot 4 0.79 m
                      (centre to the person's tag, so the robot's edge stops 2 ft away).
                      The follower slows within 6 ft of the person and won't go closer
                      than 2 ft, whatever follow_distance is set to.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DEFAULTS = {
    'tiago': {'base_frame': 'base_footprint', 'follow_distance': 0.88, 'robot_radius_m': 0.27,
              'max_linear': 1.0},
    'turtlebot4': {'base_frame': 'base_link', 'follow_distance': 0.79, 'robot_radius_m': 0.18,
                   'max_linear': 0.3},
}


def launch_nodes(context):
    robot = LaunchConfiguration('robot').perform(context)
    defaults = DEFAULTS[robot]
    follow_distance = LaunchConfiguration('follow_distance').perform(context)
    follow_distance = float(follow_distance) if follow_distance else defaults['follow_distance']
    max_linear = LaunchConfiguration('max_linear').perform(context)

    localizer = Node(
        package='people_follower', executable='uwb_localizer', output='screen',
        parameters=[{'use_sim_time': True, 'base_frame': defaults['base_frame']}])
    nodes = [localizer]
    if LaunchConfiguration('follow').perform(context) == 'true':
        nodes.append(Node(
            package='people_follower', executable='uwb_follower', output='screen',
            parameters=[{
                'use_sim_time': True, 'follow_distance': follow_distance,
                'robot_radius_m': defaults['robot_radius_m'],
                'max_linear': float(max_linear) if max_linear else defaults['max_linear'],
                'start_enabled': LaunchConfiguration('start_enabled').perform(context) == 'true',
            }]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='tiago', choices=list(DEFAULTS),
                              description='Robot being simulated'),
        DeclareLaunchArgument('follow_distance', default_value='',
                              description='Metres to stay behind the person, along their path '
                                          '(default: 0.88 for TIAGo, 0.79 for the TurtleBot 4; '
                                          'never less than the robot radius + 0.61)'),
        DeclareLaunchArgument('max_linear', default_value='',
                              description="Follower's top speed in m/s (default: 1.0 for "
                                          'TIAGo, its top speed; 0.3 for the TurtleBot 4)'),
        DeclareLaunchArgument('follow', default_value='true', choices=['true', 'false'],
                              description='false: only run the localizer'),
        DeclareLaunchArgument('start_enabled', default_value='true', choices=['true', 'false'],
                              description='false: the follower waits for ~/enable or a '
                                          'FollowPerson goal'),
        OpaqueFunction(function=launch_nodes),
    ])
