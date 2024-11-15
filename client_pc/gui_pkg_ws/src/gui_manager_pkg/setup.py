from setuptools import find_packages, setup
import os,glob
package_name = 'gui_manager_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    package_data={
        'gui_manager_pkg': ['test_gui.ui'],
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', 
         glob.glob(os.path.join('launch','*.launch.py'))),
        #  ('share/'+ package_name, )
        (os.path.join('share',package_name,'images'),[
            'resource/images/apple.png'
        ])
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
            'gui_manager = gui_manager_pkg.applecare_gui:main',
            
        ],
    },
)
