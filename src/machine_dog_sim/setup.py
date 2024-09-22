from setuptools import find_packages, setup

package_name = 'machine_dog_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='Simulates robot dog movements',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'simulator = machine_dog_sim.simulator:main',
        ],
    },
)
