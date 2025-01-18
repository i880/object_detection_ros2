from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'object_detection_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'models'), glob('models/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='i880',
    maintainer_email='ibrahimjebar255@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
	    'camera_node = object_detection_ros2.camera_node:main',
            'object_detection_node = object_detection_ros2.object_detection_node:main',
        ],
    },
)
