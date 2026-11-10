"""Detect people in the camera image with YOLO and place them relative to the robot.

Run alongside sim.launch.py. Starts:
  - person_detector: YOLO boxes on /person/detections. It imports Ultralytics, which
    lives in the virtual environment made by scripts/setup_yolo.sh, so it is started
    with that environment's Python.
  - person_localizer: box + depth image -> position in base_link on
    /person/detections_3d (system Python; it doesn't need YOLO).
  - person_tracker: smooths those over time per person (Kalman filter in odom) and
    picks the target: /person/tracks, /person/target_odom, /person/target_velocity.

    ros2 launch people_follower person_detector.launch.py
    ros2 launch people_follower person_detector.launch.py model:=/path/to/yolo11n.pt
    ros2 launch people_follower person_detector.launch.py noise:=true   # real-world errors
"""

import os

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# scripts/env.sh sets PROJECT_ROOT and YOLO_VENV. Without it, find the project from this
# file's installed location: <project>/install/people_follower/share/people_follower/launch
PROJECT_ROOT = os.environ.get(
    'PROJECT_ROOT',
    os.path.abspath(os.path.join(os.path.dirname(__file__), *['..'] * 5)))


def launch_nodes(context):
    noise = LaunchConfiguration('noise').perform(context) == 'true'
    venv_python = os.path.join(
        os.environ.get('YOLO_VENV', os.path.join(PROJECT_ROOT, '.venv-yolo')), 'bin', 'python3')
    # With noise:=true, sensor_noise sits between Gazebo and the detection nodes:
    #   camera -> sensor_noise -> person_detector -> sensor_noise -> person_localizer
    # Everything after it still reads /person/detections.
    detector = Node(
        package='people_follower', executable='person_detector', output='screen',
        prefix=venv_python,
        parameters=[{
            'use_sim_time': True,
            'model': LaunchConfiguration('model'),
            'confidence': ParameterValue(LaunchConfiguration('confidence'), value_type=float),
            'image_topic': ('/oakd_noisy/rgb/preview/image_raw' if noise
                            else '/oakd/rgb/preview/image_raw'),
            'detections_topic': '/person/detections_clean' if noise else '/person/detections',
        }])
    localizer = Node(
        package='people_follower', executable='person_localizer', output='screen',
        parameters=[{
            'use_sim_time': True,
            'depth_topic': '/oakd_noisy/rgb/preview/depth' if noise else '/oakd/rgb/preview/depth',
        }])
    tracker = Node(
        package='people_follower', executable='person_tracker', output='screen',
        parameters=[{'use_sim_time': True}])
    nodes = [detector, localizer, tracker]
    if noise:
        nodes.append(Node(
            package='people_follower', executable='sensor_noise', output='screen',
            parameters=[LaunchConfiguration('noise_params'), {'use_sim_time': True}]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model',
                              default_value=os.path.join(PROJECT_ROOT, 'models', 'yolo26n.pt'),
                              description='YOLO model file (scripts/setup_yolo.sh downloads it)'),
        DeclareLaunchArgument('confidence', default_value='0.4',
                              description='Minimum detection confidence'),
        DeclareLaunchArgument('noise', default_value='false', choices=['true', 'false'],
                              description='Add real-world camera and detection errors '
                                          '(sensor_noise, config/sensor_noise.yaml)'),
        DeclareLaunchArgument('noise_params',
                              default_value=os.path.join(
                                  get_package_share_directory('people_follower'),
                                  'config', 'sensor_noise.yaml'),
                              description='Parameters for sensor_noise'),
        OpaqueFunction(function=launch_nodes),
    ])
