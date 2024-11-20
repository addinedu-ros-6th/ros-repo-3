from setuptools import find_packages, setup
import os
import glob

package_name = 'ai_server_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob(os.path.join('launch', '*launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jh',
    maintainer_email='jh@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest']
    },
    entry_points={
        'console_scripts': [
            'obstacle = ai_server_pkg.obstacle:main',
            'pollination = ai_server_pkg.pollination:main',
            'tree_status = ai_server_pkg.tree_status:main'
        ],
    },
)
