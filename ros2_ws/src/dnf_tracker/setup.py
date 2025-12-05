from setuptools import setup
import os
from glob import glob

package_name = 'dnf_tracker'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # 👉 Install launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your@email.com',
    description='DNF Tracker package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cube_tracker_node = dnf_tracker.cube_tracker_node:main',
            'dnf_learning_node = dnf_tracker.dnf_learning_node:main',
            'dnf_recall_node = dnf_tracker.dnf_recall_node:main',
        ],
    },
)
