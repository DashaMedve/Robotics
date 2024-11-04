import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'tf2_and_carrot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='DashaMedve',
    maintainer_email='d.medvedeva@g.nsu.ru',
    description='a turtle wants a carrot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'turtle_tf2_broadcaster = tf2_and_carrot.turtle_tf2_broadcaster:main',
        'turtle_tf2_listener = tf2_and_carrot.turtle_tf2_listener:main',
        'dynamic_frame_tf2_broadcaster = tf2_and_carrot.dynamic_frame_tf2_broadcaster:main'
        ],
    },
)
