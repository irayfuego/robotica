from setuptools import setup
import os
from glob import glob

package_name = 'robotica_bringup'

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
    description='Bringup principal del robot Mecanum con Nav2 y SLAM',
    license='MIT',
    entry_points={
        'console_scripts': [],
    },
)
