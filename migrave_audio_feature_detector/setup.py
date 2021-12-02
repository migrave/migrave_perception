#!/usr/bin/env python3

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['migrave_audio_feature_detector',
              'migrave_audio_feature_detector_wrapper'],
    package_dir={'migrave_audio_feature_detector': 'common/src/migrave_audio_feature_detector',
                 'migrave_audio_feature_detector_wrapper': 'ros/src/migrave_audio_feature_detector_wrapper'}
)

setup(**d)
