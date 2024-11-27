from setuptools import setup
import os
from glob import glob

package_name = 'velocity_subscriber_xyz'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='A velocity subscriber node with serial communication.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vel_sub = velocity_subscriber_xyz.vel_sub:main',
        ],
    },
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/velocity_subscriber_xyz']),
        ('share/velocity_subscriber_xyz', ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),  # Include launch files
    ],
)

