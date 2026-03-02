from setuptools import setup
import os
from glob import glob

package_name = 'maxwell_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # ROS2用于包的注册
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # package.xml
        ('share/' + package_name, ['package.xml']),
        # 把 launch 目录下所有文件安装到系统
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='maxwell',
    maintainer_email='todo@todo.com',
    description='Launch files for maxwell robot',
    license='MIT',
    entry_points={
        'console_scripts': [],
    },
)
