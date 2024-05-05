from setuptools import setup
import os
from glob import glob

package_name = 'gap_follow'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='ahaken@ucsd.edu',
    description='Reactive Obstacle Avoidance',
    license='do whatever you want',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gap_follower_node = gap_follow.gap_follower:main'
        ],
    },
)