from setuptools import setup
import os
from glob import glob

package_name = 'robot_eyes'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'scripts'),
            glob('scripts/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='VY',
    maintainer_email='yube00@hotmail.com',
    description='Ojos animados para dos pantallas GC9A01 vía SPI',
    license='MIT',
    entry_points={
        'console_scripts': [
            'robot_eyes_node  = robot_eyes.ros_node:main',
            'huskylens_bridge = robot_eyes.huskylens_gaze_bridge:main',
            'huskylens_tts    = robot_eyes.huskylens_tts_node:main',
            'voice_command    = robot_eyes.voice_command_node:main',
        ],
    },
)
