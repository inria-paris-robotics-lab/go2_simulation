from setuptools import setup
import os
from glob import glob

package_name = 'go2_simulation'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*')),
        ('share/' + package_name + '/config', glob('config/*'))
    ],
    install_requires=['setuptools', 'pybullet'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='earlaud@inria.fr',
    description='Basic simulator wrapper to mimic real Go2 ROS2 control API.',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'simulator_node = go2_simulation.simulator_node:main',
        ],
    },
)
