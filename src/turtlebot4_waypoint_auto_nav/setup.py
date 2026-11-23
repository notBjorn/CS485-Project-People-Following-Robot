from setuptools import find_packages, setup

package_name = 'turtlebot4_waypoint_auto_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Madalynn Hein',
    maintainer_email='madalynnleehein@gmail.com',
    description='Automated Nav2 navigation for the TIAGo or TurtleBot 4',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nav_automation_node = turtlebot4_waypoint_auto_nav.nav_automation_node:main',
        ],
    },
)
