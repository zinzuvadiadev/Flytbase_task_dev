from setuptools import find_packages, setup

package_name = 'task_3'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'rclpy', 'geometry_msgs', 'turtlesim', 'turtlesim_msgs'],
    zip_safe=True,
    maintainer='dev',
    maintainer_email='zinzuvadiadev08@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'task_3 = task_3.turtle_circle:main',
            'custom_radius_publisher = task_3.publish_custom_radius:main',
        ],
    },
)
