#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    scripts=['scripts'],
    packages=['dvs_simulator_py'],
    package_dir={'': 'src'},
    install_requires=['rospy', 'rospkg', 'rosbag', 'vikit_py', 'OpenEXR', 'yaml'],
    )

setup(**d)
