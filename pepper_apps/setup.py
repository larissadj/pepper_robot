#!/usr/bin/env python
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

#d = generate_distutils_setup(
#    scripts=['nodes/camera.py', 'nodes/laser.py'],
#    packages=['pepper_apps'],
#    package_dir={'': 'src'}
#)
#setup(**d)

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['pepper_apps'],
    package_dir={'': 'src'})

setup(**setup_args)

