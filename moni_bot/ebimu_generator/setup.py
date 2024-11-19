from setuptools import find_packages, setup

package_name = 'ebimu_generator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        ('share/' + package_name + '/launch', ['launch/imu_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ksm',
    maintainer_email='0907smko@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
       		 "raw_imu_pub = ebimu_generator.raw_imu_pub:main",
       		 "madgwick_imu_pub = ebimu_generator.madgwick_imu_pub:main",
       		 "ebimu_pub = ebimu_generator.ebimu_pub:main"

	 ],
    },
)
