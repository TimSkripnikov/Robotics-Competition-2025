from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'autorace_core_ArchieRobotics'

setup(
    name=package_name,
    version='0.0.0',
    packages=['autorace_core_ArchieRobotics'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        #(os.path.join('share', package_name, 'weights'), glob('weights/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='artem',
    maintainer_email='a.skripnikov@g.nsu.ru',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'ros_competition_node = autorace_core_ArchieRobotics.ros_competition_node:main',
        'task_1_node = autorace_core_ArchieRobotics.task_1_node:main',
        'task_2_node = autorace_core_ArchieRobotics.task_2_node:main',
        'task_3_node = autorace_core_ArchieRobotics.task_3_node:main',
        'yolo_node = autorace_core_ArchieRobotics.yolo_node:main',
        'drive_node = autorace_core_ArchieRobotics.drive_node:main',
        ],
    },
)
