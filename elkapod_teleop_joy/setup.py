from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'elkapod_teleop_joy'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name, ['package.xml']))
data_files.append((os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vistek528',
    maintainer_email='piotrpatek17@gmail.com',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'elkapod_leg_joy_controller = elkapod_teleop_joy.elkapod_leg_joy_controller:main',
            'elkapod_joy_controller = elkapod_teleop_joy.elkapod_joy_controller:main'
        ],
    },
)
