"""Gazebo + TIAGo + RViz (included by sim.launch.py when robot:=tiago, the default).

The TIAGo model comes from easynav_playground_tiago_description (PAL's TIAGo,
ported to Gazebo Harmonic, with ros2_control through gz_ros2_control) and its
ROS <-> Gazebo bridge topics from easynav_playground_tiago_worlds. This file
spawns that robot into one of our worlds instead of the EasyNav small house:

  - The robot is urdf/tiago_uwb.urdf.xacro: the EasyNav TIAGo plus four UWB
    anchors on its base, for the UWB plugin and uwb_localizer.
  - Gazebo's server and GUI start separately, like turtlebot4.launch.py (started
    together, the server sometimes waited forever for the GUI), and the UWB
    ranges are bridged to ROS the same way.

  - Gazebo runs the world, with the Sensors / Imu / ForceTorque systems added
    when neither the world nor the robot already loads them (a copy of the world
    is written to a temp folder; the original is not changed).
  - robot_state_publisher + ros_gz_sim create spawn TIAGo, and spawners start
    every controller in the description's tiago_controllers.yaml.
  - The EasyNav bridge config is reused, with its world name swapped for ours,
    /clock left to our own bridge, and /scan_raw renamed to /scan.
  - tiago_base_relay maps /cmd_vel and /odom to TIAGo's mobile_base_controller,
    so the scripts and nodes see the same topics as on the TurtleBot 4.
  - SLAM (slam_toolbox), AMCL and Nav2 use their stock Jazzy parameter files
    with config/tiago/*_overrides.yaml merged on top.

    ros2 launch people_follower sim.launch.py                          # box_room
    ros2 launch people_follower sim.launch.py world:=person_world
    ros2 launch people_follower sim.launch.py slam:=true
    ros2 launch people_follower sim.launch.py localization:=true nav2:=true
"""

import glob
import os
import re
import tempfile

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml

DESCRIPTION_PKG = 'easynav_playground_tiago_description'
WORLDS_PKG = 'easynav_playground_tiago_worlds'
MODEL_NAME = 'tiago'

# Gazebo systems TIAGo's sensors need. Added to the world only when missing.
REQUIRED_SYSTEMS = [
    ('gz-sim-sensors-system',
     '<plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">'
     '<render_engine>ogre2</render_engine></plugin>'),
    ('gz-sim-imu-system',
     '<plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>'),
    ('gz-sim-forcetorque-system',
     '<plugin filename="gz-sim-forcetorque-system" name="gz::sim::systems::ForceTorque"/>'),
]

# ROS topic renames applied to the EasyNav bridge config (both robots use /scan).
ROS_TOPIC_RENAMES = {'scan_raw': 'scan', 'scan_raw/points': 'scan/points'}

# Controllers the EasyNav TIAGo starts. Others found in the controllers file
# are loaded inactive, so two controllers never fight over the same joints.
ACTIVE_CONTROLLERS = ['joint_state_broadcaster', 'mobile_base_controller',
                      'imu_sensor_broadcaster', 'ft_sensor_controller', 'torso_controller',
                      'head_controller', 'arm_controller', 'gripper_controller']

ARGUMENTS = [
    DeclareLaunchArgument('world', default_value='box_room', description='World name'),
    DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false'],
                          description='Start RViz'),
    DeclareLaunchArgument('model', default_value='standard',
                          description='Unused (TurtleBot 4 only)'),
    DeclareLaunchArgument('slam', default_value='false', choices=['true', 'false'],
                          description='Run SLAM to build a map'),
    DeclareLaunchArgument('localization', default_value='false', choices=['true', 'false'],
                          description='Run AMCL on a saved map'),
    DeclareLaunchArgument('nav2', default_value='false', choices=['true', 'false'],
                          description='Run Nav2'),
    DeclareLaunchArgument('map', default_value='',
                          description='Map yaml for localization (default: <world>.yaml)'),
]
for pose_element in ['x', 'y', 'z', 'yaw']:
    ARGUMENTS.append(DeclareLaunchArgument(pose_element, default_value='0.0',
                     description=f'{pose_element} component of the robot pose'))


def share(pkg):
    """Share directory of pkg, or None when it isn't installed."""
    try:
        return get_package_share_directory(pkg)
    except PackageNotFoundError:
        return None


def require_share(pkg):
    path = share(pkg)
    if path is None:
        raise RuntimeError(
            f"Package '{pkg}' not found. Run ./scripts/build.sh (it installs it with rosdep); "
            'if rosdep cannot find it, run ./scripts/fetch_tiago_source.sh first.')
    return path


def deep_merge(base, overrides):
    for key, value in overrides.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            deep_merge(base[key], value)
        else:
            base[key] = value
    return base


def set_in_every_node(params, values):
    """Set values in every ros__parameters block of a ROS 2 parameter file."""
    for key, value in params.items():
        if key == 'ros__parameters' and isinstance(value, dict):
            value.update(values)
        elif isinstance(value, dict):
            set_in_every_node(value, values)


def merged_params(base_file, overrides_file, out_dir, name, every_node=None):
    """Write base_file with overrides_file merged on top; return the new path."""
    with open(base_file) as f:
        params = yaml.safe_load(f) or {}
    if overrides_file and os.path.isfile(overrides_file):
        with open(overrides_file) as f:
            deep_merge(params, yaml.safe_load(f) or {})
    if every_node:
        set_in_every_node(params, every_node)
    out = os.path.join(out_dir, name)
    with open(out, 'w') as f:
        yaml.safe_dump(params, f, sort_keys=False)
    return out


def find_world(world, dirs):
    for d in dirs:
        candidate = os.path.join(d, world + '.sdf')
        if os.path.isfile(candidate):
            return candidate
    raise RuntimeError(f"World '{world}' not found in: " + ', '.join(dirs))


def prepare_world(world_file, robot_description, out_dir):
    """Copy the world, adding the sensor systems TIAGo needs; return (path, world name)."""
    with open(world_file) as f:
        sdf = f.read()
    match = re.search(r'<world\s+name\s*=\s*["\']([^"\']+)["\']\s*>', sdf)
    if not match:
        raise RuntimeError(f'No <world name="..."> in {world_file}')
    missing = [xml for key, xml in REQUIRED_SYSTEMS
               if key not in sdf and key not in robot_description]
    if missing:
        sdf = sdf[:match.end()] + '\n' + '\n'.join(missing) + '\n' + sdf[match.end():]
    out = os.path.join(out_dir, os.path.basename(world_file))
    with open(out, 'w') as f:
        f.write(sdf)
    return out, match.group(1)


def bridge_entries(worlds_share):
    files = sorted(glob.glob(os.path.join(worlds_share, 'config', 'bridge', '**', '*.yaml'),
                             recursive=True))
    if not files:
        files = sorted(glob.glob(os.path.join(worlds_share, '**', '*bridge*.yaml'),
                                 recursive=True))
    entries = []
    for path in files:
        with open(path) as f:
            data = yaml.safe_load(f)
        if isinstance(data, list):
            entries += [e for e in data if isinstance(e, dict)]
    if not entries:
        raise RuntimeError(f'No ROS <-> Gazebo bridge config found in {worlds_share}')
    return entries


def prepare_bridge(worlds_share, world_name, out_dir):
    """Rewrite the EasyNav bridge config for our world; return its path."""
    result = []
    for entry in bridge_entries(worlds_share):
        entry = dict(entry)
        ros = entry.pop('ros_topic_name', None) or entry.get('topic_name')
        gz = entry.pop('gz_topic_name', None) or entry.get('topic_name')
        entry.pop('topic_name', None)
        if not ros or not gz or ros.strip('/') == 'clock':
            continue  # our clock_bridge already bridges /clock
        bare = ros.strip('/')
        ros = ('/' if ros.startswith('/') else '') + ROS_TOPIC_RENAMES.get(bare, bare)
        gz = re.sub(r'/world/[^/]+/', f'/world/{world_name}/', gz)
        entry['ros_topic_name'] = ros
        entry['gz_topic_name'] = gz
        result.append(entry)
    out = os.path.join(out_dir, 'tiago_bridge.yaml')
    with open(out, 'w') as f:
        yaml.safe_dump(result, f, sort_keys=False)
    return out


def controller_names(description_share):
    path = os.path.join(description_share, 'config', 'tiago_controllers.yaml')
    if not os.path.isfile(path):
        return ACTIVE_CONTROLLERS, []
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    cm = (data.get('controller_manager') or {}).get('ros__parameters') or {}
    names = [k for k, v in cm.items() if isinstance(v, dict) and 'type' in v]
    if not names:
        return ACTIVE_CONTROLLERS, []
    return ([n for n in ACTIVE_CONTROLLERS if n in names],
            [n for n in names if n not in ACTIVE_CONTROLLERS])


def find_map(context, world):
    map_yaml = LaunchConfiguration('map').perform(context)
    if not map_yaml:
        for pkg in ['people_follower', 'turtlebot4_navigation']:
            pkg_share = share(pkg)
            if pkg_share is None:
                continue
            candidate = os.path.join(pkg_share, 'maps', world + '.yaml')
            if os.path.isfile(candidate):
                map_yaml = candidate
                break
    if not map_yaml:
        raise RuntimeError(
            'No map found for this world. Build one with slam:=true and scripts/save_map.sh, '
            'or pass map:=/path/to/map.yaml')
    return map_yaml


def launch_setup(context):
    import xacro  # ros-jazzy-xacro; imported here so a missing package gives a clear error

    def arg(name):
        return LaunchConfiguration(name).perform(context)

    world = arg('world')
    slam = arg('slam') == 'true'
    localization = arg('localization') == 'true'
    nav2 = arg('nav2') == 'true'
    if slam and localization:
        raise RuntimeError('Use slam:=true (build a map) or localization:=true (use one), not both')

    pkg_people_follower = get_package_share_directory('people_follower')
    pkg_description = require_share(DESCRIPTION_PKG)
    pkg_worlds = require_share(WORLDS_PKG)
    pkg_tb4_gz_bringup = share('turtlebot4_gz_bringup')
    out_dir = tempfile.mkdtemp(prefix='people_follower_tiago_')

    # Robot model: the EasyNav TIAGo plus UWB anchors 1-4 on the base
    urdf_file = os.path.join(pkg_people_follower, 'urdf', 'tiago_uwb.urdf.xacro')
    robot_description = xacro.process_file(urdf_file).toxml()

    # World: ours first, then the TB4 and EasyNav worlds
    world_dirs = [os.path.join(pkg_people_follower, 'worlds')]
    if pkg_tb4_gz_bringup:
        world_dirs.append(os.path.join(pkg_tb4_gz_bringup, 'worlds'))
    world_dirs.append(os.path.join(pkg_worlds, 'worlds'))
    world_file, world_name = prepare_world(find_world(world, world_dirs),
                                           robot_description, out_dir)

    resource_dirs = world_dirs + [
        os.path.dirname(pkg_description),        # package://easynav_playground_tiago_description
        os.path.dirname(pkg_worlds),
        os.path.join(pkg_worlds, 'models'),
    ]
    for pkg in ['turtlebot4_description', 'irobot_create_description']:
        if share(pkg):
            resource_dirs.append(os.path.dirname(share(pkg)))
    existing = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    gz_resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', ':'.join(resource_dirs + ([existing] if existing else [])))

    # Server and GUI started separately (see turtlebot4.launch.py), sensors on ogre2.
    # -v 4: scripts/sim_start.sh waits for lines Gazebo only prints at this level.
    gz_sim_launch = PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'))
    gazebo_server = IncludeLaunchDescription(
        gz_sim_launch,
        launch_arguments=[('gz_args', f'{world_file} -s -r -v 4 --render-engine-server ogre2')])
    gazebo_gui = IncludeLaunchDescription(
        gz_sim_launch, launch_arguments=[('gz_args', '-g -v 4')])

    clock_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='clock_bridge',
        output='screen', arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'])

    # UWB ranges from anchors 1-4 to tag 1 (see turtlebot4.launch.py)
    uwb_topics = [f'/uwb_gz_simulator/{kind}/a{anchor}t1'
                  for kind in ('distances', 'distances_ground_truth') for anchor in range(1, 5)]
    uwb_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='uwb_bridge',
        output='screen',
        arguments=[f'{topic}@std_msgs/msg/Float64[gz.msgs.Double' for topic in uwb_topics])

    tiago_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='tiago_bridge',
        output='screen',
        parameters=[{'config_file': prepare_bridge(pkg_worlds, world_name, out_dir),
                     'use_sim_time': True}])

    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher', output='screen',
        parameters=[{'robot_description': ParameterValue(robot_description, value_type=str),
                     'use_sim_time': True}])

    spawn = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-world', world_name, '-topic', 'robot_description', '-name', MODEL_NAME,
                   '-x', arg('x'), '-y', arg('y'), '-z', arg('z'), '-Y', arg('yaw')])

    # Gazebo can take a while to start, so the spawners wait up to 2 minutes for it
    active, inactive = controller_names(pkg_description)
    spawner_args = ['--controller-manager', '/controller_manager',
                    '--controller-manager-timeout', '120']
    spawners = [Node(package='controller_manager', executable='spawner', output='screen',
                     arguments=[name] + spawner_args) for name in active]
    spawners += [Node(package='controller_manager', executable='spawner', output='screen',
                      arguments=[name, '--inactive'] + spawner_args) for name in inactive]

    base_relay = Node(package='people_follower', executable='tiago_base_relay', output='screen',
                      parameters=[{'use_sim_time': True}])

    actions = [
        LogInfo(msg=f'TIAGo in world "{world_name}" ({world_file}); '
                    f'generated configs in {out_dir}'),
        gz_resource_path, gazebo_server, gazebo_gui, clock_bridge, uwb_bridge, tiago_bridge,
        robot_state_publisher, spawn,
        *spawners, base_relay,
    ]

    tiago_config = os.path.join(pkg_people_follower, 'config', 'tiago')
    pkg_nav2_bringup = share('nav2_bringup')

    if slam:
        pkg_slam = get_package_share_directory('slam_toolbox')
        slam_params = merged_params(
            os.path.join(pkg_slam, 'config', 'mapper_params_online_async.yaml'),
            os.path.join(tiago_config, 'slam_overrides.yaml'), out_dir, 'slam.yaml',
            every_node={'use_sim_time': True})
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                pkg_slam, 'launch', 'online_async_launch.py')),
            launch_arguments=[('slam_params_file', slam_params), ('use_sim_time', 'true')]))

    if localization or nav2:
        nav2_params = merged_params(
            os.path.join(pkg_nav2_bringup, 'params', 'nav2_params.yaml'),
            os.path.join(tiago_config, 'nav2_overrides.yaml'), out_dir, 'nav2.yaml',
            every_node={'use_sim_time': True, 'enable_stamped_cmd_vel': True})
        if localization:
            actions.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    pkg_nav2_bringup, 'launch', 'localization_launch.py')),
                launch_arguments=[('map', find_map(context, world)), ('params_file', nav2_params),
                                  ('use_sim_time', 'true'), ('autostart', 'true')]))
        if nav2:
            actions.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    pkg_nav2_bringup, 'launch', 'navigation_launch.py')),
                launch_arguments=[('params_file', nav2_params), ('use_sim_time', 'true'),
                                  ('autostart', 'true')]))

    if arg('rviz') == 'true':
        # Nav2's RViz view (robot, laser, map, costmaps, goal tools). Without a map,
        # odom is the fixed frame.
        with open(os.path.join(pkg_nav2_bringup, 'rviz', 'nav2_default_view.rviz')) as f:
            rviz_config = f.read()
        if not (slam or localization):
            rviz_config = rviz_config.replace('Fixed Frame: map', 'Fixed Frame: odom')
        rviz_file = os.path.join(out_dir, 'tiago.rviz')
        with open(rviz_file, 'w') as f:
            f.write(rviz_config)
        actions.append(Node(package='rviz2', executable='rviz2', output='screen',
                            arguments=['-d', rviz_file], parameters=[{'use_sim_time': True}]))

    return actions


def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
