import os
from glob import glob
from setuptools import setup

package_name = 'ground_truth_mapping'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # Include our package.xml file
        (os.path.join('share', package_name), ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Enrico Piazza',
    maintainer_email='enrico.piazza@polimi.it',
    description='Execute benchmark of ROS localization packages to build a performance model',
    license='BSD License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'execute_ground_truth_mapping = ground_truth_mapping.execute_ground_truth_mapping:main',
            'ground_truth_mapping_supervisor = ground_truth_mapping.ground_truth_mapping_supervisor:main',
        ],
    },
)
