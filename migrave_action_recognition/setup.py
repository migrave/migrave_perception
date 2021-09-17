#!/usr/bin/env python3

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['migrave_action_reconition',
              'migrave_action_recognition_wrapper'],
    package_dir={'migrave_action_recognition': 'common/src/migrave_action_recognition',
                 'migrave_action_recognition_wrapper': 'ros/src/migrave_action_recognition_wrapper'}
)

setup(**d)
