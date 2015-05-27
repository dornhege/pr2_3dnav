from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup()
d['packages'] = ['pr2_3dnav_visualization']
d['scripts'] = []
d['package_dir'] = {'': 'python'}
setup(**d)
