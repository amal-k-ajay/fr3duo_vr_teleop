import os
from glob import glob
from setuptools import setup

package_name = 'fr3duo_quest_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Include all config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'scipy', 'pure-python-adb'],
    zip_safe=True,
    maintainer='Amal',
    maintainer_email='amalkajayagosh@gmail.com',
    description='Dual Franka FR3 teleoperation via Meta Quest 3',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'franka_teleop_node = fr3duo_quest_teleop.franka_teleop_node:main',
            'oculus_bridge_node = fr3duo_quest_teleop.oculus_bridge_node:main',
        ],
    },
)
