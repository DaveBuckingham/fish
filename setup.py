from setuptools import setup, find_packages
from codecs import open
import os
import sys

VERSION = '0.1'

here = os.path.abspath(os.path.dirname(__file__))

package_name = 'imucapture'

# Get the long description from the README file
with open(os.path.join(here, 'README.rst'), encoding='utf-8') as f:
    long_description = f.read()

with open(os.path.join(here, package_name, 'VERSION')) as version_file:
    read_version = version_file.read().strip()



setup(
    name='IMUCapture',

    # Versions should comply with PEP440.  For a discussion on single-sourcing
    # the version across setup.py and the project code, see
    # https://packaging.python.org/en/latest/single_source_version.html

    version = VERSION

    description='Collect data from IMUs via Arduino',
    long_description=long_description,

    # The project's main homepage.
    url='https://github.com/DaveBuckingham/fish',

    # Author details
    author='Tytell Lab at Tufts University',
    author_email='pypa-dev@googlegroups.com',

    # Choose your license
    license='GPLv3',

    # See https://pypi.python.org/pypi?%3Aaction=list_classifiers
    classifiers=[
        # How mature is this project? Common values are
        #   3 - Alpha
        #   4 - Beta
        #   5 - Production/Stable
        'Development Status :: 3 - Alpha',

        # Indicate who your project is intended for
        'Intended Audience :: Biologists',
        'Topic :: Kinematics collection :: IMU tracking',

        # Pick your license as you wish (should match "license" above)
        'License :: OSI Approved :: GPLv3 License',

        # Specify the Python versions you support here. In particular, ensure
        # that you indicate whether you support Python 2, Python 3 or both.
        'Programming Language :: Python :: 3.4',
        'Programming Language :: Python :: 3.5',
    ],

    # What does your project relate to?
    keywords='IMU, fish, kinematics',

    # You can just specify the packages manually here if your project is
    # simple. Or you can use find_packages().
    packages=[package_name],

    # Alternatively, if you want to distribute just a my_module.py, uncomment
    # this:
    #   py_modules=["my_module"],


    # List run-time dependencies here.  These will be installed by pip when
    # your project is installed. For an analysis of "install_requires" vs pip's
    # requirements files see:
    # https://packaging.python.org/en/latest/requirements.html

    install_requires=['h5py>=2.7.0', 
                      'pyqtgraph>=0.10.0',
                      'pyserial>=3.3',
                      'PyQt5>=5.8.2',
                      'pyquaternion>=0.9.2',
                     ],


    # List additional groups of dependencies here (e.g. development
    # dependencies). You can install these using the following syntax,
    # for example:
    # $ pip install -e .[dev,test]
    # extras_require={
    #     'dev': ['check-manifest'],
    #     'test': ['coverage'],
    # },

    # If there are data files included in your packages that need to be
    # installed, specify them here.  If using Python 2.6 or less, then these
    # have to be included in MANIFEST.in as well.
    package_data={'': ['VERSION'], },
    include_package_data=True,

    # Although 'package_data' is the preferred approach, in some case you may
    # need to place data files outside of your packages. See:
    # http://docs.python.org/3.4/distutils/setupscript.html#installing-additional-files # noqa
    # In this case, 'data_file' will be installed into '<sys.prefix>/my_data'
    # data_files=[('my_data', ['data/data_file'])],

    # To provide executable scripts, use entry points in preference to the
    # "scripts" keyword. Entry points provide cross-platform support and allow
    # pip to create the appropriate form of executable for the target platform.
    # entry_points={
    #     'console_scripts': [
    #         'app=fish.__main__:main',
    #     ],
    # },
    entry_points={
        'gui_scripts': [
            'imucapture=imucapture.__main__:main',
        ],
    },

)
