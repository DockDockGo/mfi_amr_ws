from setuptools import setup
import os
from glob import glob

package_name = 'sensor_health_monitor'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='soham',
    maintainer_email='sohambhave1998@gmail.com',
    description='Package to monitor and publish sensor health',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_health_monitor = sensor_health_monitor.sensor_health_monitor:main',
        ],
    },
)
