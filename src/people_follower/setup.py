from glob import glob

from setuptools import find_packages, setup

package_name = 'people_follower'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
        ('share/' + package_name + '/models', glob('models/*.sdf')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/maps', glob('maps/*.yaml') + glob('maps/*.pgm')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Julian Contreras',
    maintainer_email='floatingbowl@gmail.com',
    description='CS485 person-following robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'follower = people_follower.follower:main',
            'human_waypoints = people_follower.human_waypoints:main',
            'person_detector = people_follower.person_detector:main',
            'person_localizer = people_follower.person_localizer:main',
            'sensor_noise = people_follower.sensor_noise:main',
            'person_tracker = people_follower.person_tracker:main',
            'person_follower = people_follower.person_follower:main',
            'safety_guard = people_follower.safety_guard:main',
            'auto_explorer = people_follower.auto_explorer:main',
        ],
    },
)
