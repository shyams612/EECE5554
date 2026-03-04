from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'imu_driver'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='ROS2 IMU driver for VectorNav VN-100',
    license='MIT',
    entry_points={
        'console_scripts': [
            'imu_driver = imu_driver.imu_driver:main',
        ],
    },
)