#!/usr/bin/env python3

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=["migrave_action_recognition", "action_recognition"],
    package_dir={
        "migrave_action_recognition": "ros/src/migrave_action_recognition",
        "action_recognition": "common/src/action_recognition",
    },
)

setup(**d)
