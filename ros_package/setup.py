from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup(**generate_distutils_setup(packages=['PACKAGE_NAME'],
                                 scripts=['scripts/ADD_PY_NODE'],
                                 package_dir={'': 'src'}))
