from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'system_monitor'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dev',
    maintainer_email='dev@todo.todo',
    description='System monitoring node for ROS2 workspace',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'system_monitor_node = system_monitor.system_monitor_node:main',
            'monitoring_dashboard = system_monitor.monitoring_dashboard:main',
        ],
    },
)
