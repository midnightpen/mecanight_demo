from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'bogie_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch'),glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mecanight',
    maintainer_email='mecanight@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "base_controller = bogie_bringup.base_controller:main",
            "calibrate = bogie_bringup.calibrate:main",
        ],
    },
)
