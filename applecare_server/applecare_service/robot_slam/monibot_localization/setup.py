from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'monibot_localization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
               
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),

        # Include configuration files (e.g., Cartographer .lua files)
        (os.path.join('share', package_name, 'config'), glob('config/*.lua')),

        # Include parameter files
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'map'), glob('map/*.yaml')),
        (os.path.join('share', package_name, 'map'), glob('map/*.pgm')),


    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ksm',
    maintainer_email='0907smko@gmail.com',
    description='ROS2 SLAM (mainly localization) package for applecare service',
    license='Apache License 2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            # mapping (custom. unsuccessful. use cartographer)
            # 'scan_matching = monibot_localization.scan_matching:main',
            # localization
            'ekf_filter = monibot_localization.ekf_filter:main',
            'custom_amcl = monibot_localization.custom_amcl:main',
            # amcl test nodes
            # 'simple_amcl_1 = monibot_localization.simple_amcl_1:main',
            # 'simple_amcl_2 = monibot_localization.simple_amcl_2:main',
            # loading static map
            'map_publisher = monibot_localization.map_publisher:main',

        ],
    },
)
