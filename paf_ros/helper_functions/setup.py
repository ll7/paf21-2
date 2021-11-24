"""
Setup for helper_functions
"""

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['helper_functions'],
    package_dir={'': 'src'}
)

setup(**d)
