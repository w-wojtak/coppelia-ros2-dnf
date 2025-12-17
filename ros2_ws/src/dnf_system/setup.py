from setuptools import setup
import os
from glob import glob

package_name = 'dnf_system'

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
    maintainer='Weronika Wojtak',
    maintainer_email='weronika.wojtak@ccg.pt',
    description='DNF System package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cube_input_node = dnf_system.cube_input_node:main',
            'dnf_learning_node = dnf_system.dnf_learning_node:main',
            'dnf_recall_node = dnf_system.dnf_recall_node:main',
        ],
    },
)
