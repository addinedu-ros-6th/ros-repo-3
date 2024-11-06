from setuptools import find_packages, setup
import glob
import os

package_name = 'applecare_gui_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob(os.path.join('launch', '*.launch.py'))),
    ],
    package_data={
        'applecare_gui_pkg': ['resources/*'],  # 리소스 파일을 포함
    },
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zeki',
    maintainer_email='kyeongminlee99@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gui_main = applecare_gui_pkg.gui_main:main',
        ],
    },
)
