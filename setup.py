#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# for your packages to be recognized by python
d = generate_distutils_setup(
 packages=['ground_truth_mapping_ros'],
 package_dir={'ground_truth_mapping_ros': 'src/ground_truth_mapping_ros'}
)

setup(**d)
