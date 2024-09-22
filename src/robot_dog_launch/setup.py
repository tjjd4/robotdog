from setuptools import find_packages, setup
import os 
from glob import glob

package_name = 'robot_dog_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='Launch package for robot_dog',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [],
    },
)
