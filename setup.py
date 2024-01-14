from setuptools import setup
import os
from glob import glob

package_name = 'akros2_base'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config', 'akros2_mecanum'), glob('config/akros2_mecanum/*.yaml')),
        (os.path.join('share', package_name, 'config', 'akros2_omni'), glob('config/akros2_omni/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aditya Kamath',
    maintainer_email='adityakamath@live.com',
    description='AKROS2 base files for drivers, sensors, filters and fusion',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motion_detector = akros2_base.motion_detector:main',
        ],
    },
)
