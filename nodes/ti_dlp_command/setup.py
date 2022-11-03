from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
d = generate_distutils_setup(
    packages=['ti_dlp_command'],
    package_dir={'': 'src'}
)
setup(**d)
