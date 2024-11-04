from setuptools import find_packages, setup
import glob
import os
package_name = 'monibot_chatter'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',glob.glob(os.path.join('launch', '*.launch.*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zeki',
    maintainer_email='kyeongminlee99@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tmp_chatter_publish = monibot_chatter.tmp_chatter_publish:main',
            'tmp_chatter_subcribe = monibot_chatter.tmp_chatter_subscribe:main',
        ],
    },
)
