from setuptools import find_packages, setup
from glob import glob

package_name = 'dsr_practice'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/config',
         glob('config/*.yaml') + glob('dsr_practice/Calibration_Tutorial/*.npy')
         ),
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
        'mp_basic = dsr_practice.mp_basic:main',
        'mp_waypoint = dsr_practice.mp_waypoint:main',
        'mp_waypoint_pilz = dsr_practice.mp_waypoint_pilz:main',
        'collision_obstacle = dsr_practice.collision_obstacle:main',
        'gripper = dsr_practice.gripper:main',
        'pick_and_place = dsr_practice.pick_and_place:main',
        'click_pick_node= dsr_practice.click_pick_node:main',
        'click_pick_two = dsr_practice.click_pick_two:main',
        'stt_node = dsr_practice.stt_node:main',
        'stt_robot_control = dsr_practice.stt_robot_control:main',
        'stt_pick_and_place = dsr_practice.stt_pick_and_place:main',
        ],
    },
)
