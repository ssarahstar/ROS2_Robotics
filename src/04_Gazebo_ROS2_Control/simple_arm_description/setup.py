from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'simple_arm_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
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
            'joint_monitor = simple_arm_description.joint_monitor:main',
            'arm_sequencer = simple_arm_description.arm_sequence:main',
            'auto_stop = simple_arm_description.auto_stop:main', 
            'trajectory_publisher = simple_arm_description.trajectory_publisher:main',
            'state_monitor = simple_arm_description.state_monitor:main',
            'waypoint_sequencer = simple_arm_description.waypoint_sequencer:main',
        ],
    },
)
