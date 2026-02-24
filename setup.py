import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'qcar2_mixer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hackbrian',
    maintainer_email='brayandurantoconas@gmail.com',
    description='QCar2 final motor command mixer package with safety logic.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'qcar2_mixer_node = qcar2_mixer.mixer_node:main',
            'led_sequence_node = qcar2_mixer.led_sequence_node:main',
        ],
    },
)
