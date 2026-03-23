from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'gap_explorer'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lmkonzen',
    maintainer_email='your_email@example.com',
    description='Detect map openings and send Nav2 goals for exploration/probing.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gap_explorer = gap_explorer.gap_explorer_node:main',
        ],
    },
)