from setuptools import setup
import os
from glob import glob

package_name = 'robot_publisher_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/urdf', glob('urdf/*.urdf')),  # Sørg for at URDF er inkludert her
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Publishes Twist messages to control the robot.',
    license='License declaration',
    entry_points={
        'console_scripts': [
            'robot_publisher = robot_publisher_pkg.robot_publisher:main',
        ],
    },
)
