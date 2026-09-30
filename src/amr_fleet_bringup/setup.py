from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'amr_fleet_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Raghav Gangrade',
    maintainer_email='raghavgangrade72009@gmail.com',
    description='Launch scripts and fleet bringup scaffolding.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'warehouse_visualizer = amr_fleet_bringup.warehouse_visualizer:main',
        ],
    },
)

