from setuptools import setup

package_name = 'odrive_ros2'

setup(
    name=package_name,
    version='0.1.2',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='martin',
    author_email='martma18@uia.no',
    description='ROS 2 interface for ODrive.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odrive_ros2 = odrive_ros2.odrive_ros2:main'
        ],
    },
)