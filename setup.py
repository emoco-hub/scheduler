from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'scheduler'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=[
        'setuptools',
    ],
    zip_safe=True,
    maintainer='Emoco Labs',
    maintainer_email='support@emoco.com',
    description='ROS2 scheduler node for controlling relays based on time, sun position, and relay state conditions',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scheduler_node = scheduler.scheduler_node:main',
        ],
    },
)
