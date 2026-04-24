from setuptools import find_packages, setup

package_name = 'simple_arm_trajectory'

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
    maintainer='ssu',
    maintainer_email='ssu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'controller_state_monitor = simple_arm_trajectory.controller_state_monitor:main',
        'send_waypoint = simple_arm_trajectory.send_waypoint:main',
        'waypoint_action_follower = simple_arm_trajectory.waypoint_action_follower:main',
        'trajectory_monitor = simple_arm_trajectory.trajectory_monitor:main',
        'repeat_monitor = simple_arm_trajectory.repeat_monitor:main',
        ],
    },
)
