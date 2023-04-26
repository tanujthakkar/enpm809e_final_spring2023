from setuptools import setup
import os
from glob import glob

package_name = 'rwa4_group19'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tanujthakkar',
    maintainer_email='tanujthakkar720@gmail.com',
    description='RWA4 Group19',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rwa4 = rwa4_group19.rwa4:main',
        ],
    },
)