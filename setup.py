from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

import os

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['control'],
    package_dir={'': 'src'})

setup(**setup_args)