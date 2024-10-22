from setuptools import setup
import os
from glob import glob

package_name = 'lidar_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=['lidar_package'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Inkluder launch-filer her
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='adnan',
    maintainer_email='adnan@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
