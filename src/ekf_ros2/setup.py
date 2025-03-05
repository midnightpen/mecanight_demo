from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ekf_ros2'

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
            "ekf_3d = ekf_ros2.ekf_3d:main",
            "ekf_6d = ekf_ros2.ekf_6d:main",
            "ekf_fuzzy_R_6d = ekf_ros2.ekf_fuzzy_6d:main",
            "ekf_realsense = ekf_ros2.ekf_realsense:main",
            "odom_test = ekf_ros2.ekf_odom_test:main",
            "calibrate_ekf = ekf_ros2.calibrate_ekf:main",
            "ekf_6d_omni_test = ekf_ros2.ekf_6d_omni_test:main",
        ],
    },
)
