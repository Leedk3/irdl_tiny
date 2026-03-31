from setuptools import setup
import os
from glob import glob

package_name = 'lidar_mapping'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'open3d', 'numpy'],
    zip_safe=True,
    maintainer='YourName',
    maintainer_email='yourmail@etri.re.kr',
    description='LiDAR mapping demo with ad_viz_node',
    license='BSD',
    entry_points={
        'console_scripts': [
            'lidar_sim_node = lidar_mapping.lidar_sim_node:main',
            'map_builder_node = lidar_mapping.map_builder_node:main',
        ],
    },
)
