from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'so101_servo_dynamics'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='moulie',
    maintainer_email='moulie@local.dev',
    description='SO101 Servo Dynamics Characterization and Real-Time Web Control',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hardware_connection = so101_servo_dynamics.hardware_connection_node:main',
            'servo_characterization = so101_servo_dynamics.servo_characterization_node:main',
            'web_control_bridge = so101_servo_dynamics.web_control_bridge:main',
        ],
    },
)
