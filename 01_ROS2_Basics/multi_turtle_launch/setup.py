from setuptools import find_packages, setup
import os
from glob import glob



package_name = 'multi_turtle_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', 'multi_turtle_launch', 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
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
            'speed_controller = multi_turtle_launch.speed_controller:main',
        ],
    },
)
