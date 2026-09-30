from setuptools import find_packages, setup

package_name = 'amr_fleet_core'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Raghav Gangrade',
    maintainer_email='raghavgangrade72009@gmail.com',
    description='Core interface abstractions and state management for AMR fleet coordination.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'task_manager = amr_fleet_core.task_manager_node:main',
            'cbba_node = amr_fleet_core.cbba_node:main',
            'rh_node = amr_fleet_core.rh_node:main',
        ],
    },
)

