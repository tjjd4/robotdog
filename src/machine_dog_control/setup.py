from setuptools import find_packages, setup

package_name = 'machine_dog_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    py_modules=['machine_dog_control.controller'],
    install_requires=['setuptools', 'Flask','Werkzeug'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='your_email@example.com',
    description='Example robot dog controller',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'machine_dog_control = machine_dog_control.controller:main',
        ],
    },
)
