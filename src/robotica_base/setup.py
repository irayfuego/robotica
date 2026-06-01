from setuptools import setup
import os
from glob import glob

package_name = 'robotica_base'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Robot Autonomo',
    maintainer_email='pi@robotica.local',
    description='Controlador base del robot',
    license='MIT',
    entry_points={
        'console_scripts': [
            'base_node = robotica_base.base_node:main',
            'odometry_node = robotica_base.odometry_node:main',
        ],
    },
)
