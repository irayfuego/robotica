from setuptools import setup
import os
from glob import glob

package_name = 'rrc_lite_ros2'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='VY',
    maintainer_email='yube00@hotmail.com',
    description='Driver ROS 2 para placa controladora Huaner/Hiwonder STM32 (mecanum)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'rrc_lite_node = rrc_lite_ros2.rrc_lite_node:main',
        ],
    },
)
