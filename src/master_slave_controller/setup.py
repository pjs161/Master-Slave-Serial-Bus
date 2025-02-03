from setuptools import find_packages, setup

package_name = 'master_slave_controller'

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
    maintainer='pjs161',
    maintainer_email='pjs161@todo.todo',
    description='Master-Slave Robot Arm Controller using ROS2 and Dynamixel SDK',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'teleop_controller = master_slave_controller.teleop_controller:main',
            	'slave_controller = master_slave_controller.slave_controller:main',
        ],
    },
)
