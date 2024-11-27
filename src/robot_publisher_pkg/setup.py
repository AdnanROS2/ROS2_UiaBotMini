from setuptools import setup
from glob import glob
import os

package_name = 'robot_publisher_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # Resource index
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # Package manifest
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # URDF files
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Robot Publisher Package',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'robot_publisher = robot_publisher_pkg.robot_publisher:main',
            'custom_joint_state_publisher = robot_publisher_pkg.custom_joint_state_publisher:main',
        ],
    },
)

