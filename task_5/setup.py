from setuptools import find_packages, setup

package_name = 'task_5'

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
            'task_5 = task_5.nerfed_police_turtle:main',
        ],
    },
)
