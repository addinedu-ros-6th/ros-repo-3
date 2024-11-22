from setuptools import find_packages, setup

package_name = 'pollibot_arduino'

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
    maintainer='ask',
    maintainer_email='dldmsgnl1212@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_motor_control = pollibot_arduino.serial_motor_control:main',
            'move_to_ready_pose = pollibot_moveit_config.move_to_ready_pose:main',
        ],
    },
)
