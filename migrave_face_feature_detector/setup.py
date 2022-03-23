#!/usr/bin/env python3

from distutils.core import setup

from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['migrave_face_feature_detector',
              'face_recognition',
             ],
    package_dir={'migrave_face_feature_detector': 'common/src/migrave_face_feature_detector',
                 'face_recognition': 'common/src/face_recognition',
                }
)

setup(**d)
