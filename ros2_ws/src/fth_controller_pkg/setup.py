from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'fth_controller_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml') + glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Fatih Dursun',
    maintainer_email='fatih.dursun@manchester.ac.uk',
    description='This package is for controlling the robot.',
    license='LGPL',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [           
            'robot_controller = fth_controller_pkg.robot_controller_node:main',
            'camera_tf_publisher = fth_controller_pkg.camera_tf_publisher_node:main',
        ],
    },
)
