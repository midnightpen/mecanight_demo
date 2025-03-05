from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'bogie_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch'),glob('launch/*.launch.py')),
        (os.path.join('share',package_name,'config'),glob('config/*.lua')),
        (os.path.join('share',package_name,'config'),glob('config/*.yaml')),
        (os.path.join('share',package_name,'config'),glob('config/*.pgm')),
        (os.path.join('share',package_name,'config'),glob('config/*.rviz')),
        (os.path.join('share',package_name,'rviz'),glob('rviz/*.rviz')),
        (os.path.join('share',package_name,'maps'),glob('maps/*.yaml')),
        (os.path.join('share',package_name,'maps'),glob('maps/*.pgm')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mountain',
    maintainer_email='mountain@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "teleop = bogie_navigation.teleop:main",
            "nav2_cmd = bogie_navigation.navigation_command:main",
            "webcam = bogie_navigation.webcam:main",
            "qr_code = bogie_navigation.qr_code:main",
            "led = bogie_navigation.led:main",
            "led_status = bogie_navigation.led_status:main",
            "pub_point = bogie_navigation.pub_str_point:main",
            "nav_com = bogie_navigation.nav_command_test:main",
            "sub_point = bogie_navigation.sub_str_point:main",
            "sub2_point = bogie_navigation.sub_str_point_copy:main",
            "cafe = bogie_navigation.cafe_night:main",
            "teleop_omni = bogie_navigation.teleop_holomonic:main",
        ],
    },
)
