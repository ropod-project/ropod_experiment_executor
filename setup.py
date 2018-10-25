#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['ropod_experiment_executor', 'experiment_executor'],
    package_dir={'ropod_experiment_executor': 'ros/src/ropod_experiment_executor',
                 'experiment_executor': 'common/experiment_executor'}
)

setup(**d)
