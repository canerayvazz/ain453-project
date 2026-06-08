import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'pf_localization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'),
         glob(os.path.join('urdf', '*'))),
        (os.path.join('share', package_name, 'meshes'),
         glob(os.path.join('meshes', '*.dae'))),
        (os.path.join('share', package_name, 'worlds'),
         glob(os.path.join('worlds', '*.world'))),
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*.py'))),
        (os.path.join('share', package_name, 'config'),
         glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'rviz'),
         glob(os.path.join('rviz', '*.rviz'))),
        (os.path.join('share', package_name, 'materials', 'scripts'),
         glob(os.path.join('materials', 'scripts', '*'))),
        (os.path.join('share', package_name, 'materials', 'textures'),
         glob(os.path.join('materials', 'textures', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='PF Localization',
    maintainer_email='noreply@example.com',
    description='Particle filter localization for Duckiebot in Gazebo',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pf_node = pf_localization.pf_node:main',
        ],
    },
)
