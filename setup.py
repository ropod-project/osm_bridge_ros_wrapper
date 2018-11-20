#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
   packages=['osm_bridge_ros_wrapper'],
   package_dir={'osm_bridge_ros_wrapper': 'ros/src/osm_bridge_ros_wrapper'}
)

setup(**d)
