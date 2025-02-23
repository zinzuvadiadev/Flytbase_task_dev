from setuptools import setup
import os
from glob import glob

package_name = 'task_1'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'rclpy', 'geometry_msgs', 'turtlesim', 'turtlesim_msgs'],
    zip_safe=True,
    maintainer='dev',
    maintainer_email='your_email@example.com',
    description='Task 1 for FlytBase',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'task_1 = task_1.turtle_goal:main',
        ],
    },
)
