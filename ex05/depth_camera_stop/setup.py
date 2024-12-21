from glob import glob
from setuptools import find_packages, setup

package_name = 'depth_camera_stop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='DashaMedve',
    maintainer_email='d.medvedeva@g.nsu.ru',
    description='I am learning to stop my robot behind obstacles using the data from the depth camera',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'move = depth_camera_stop.move_or_stop:main'
        ],
    },
)
