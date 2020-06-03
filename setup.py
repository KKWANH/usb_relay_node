from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['usb_relay_wsn'],
    package_dir={'': 'src'}
)

setup(**d)