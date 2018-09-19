from setuptools import setup, find_packages
from codecs import open
import os
import sys

from imucapture.global_data import Global_data

VERSION = '0.1'

here = os.path.abspath(os.path.dirname(__file__))

package_name = 'imucapture'

# Get the long description from the README file
with open(os.path.join(here, 'README.rst'), encoding='utf-8') as f:
    long_description = f.read()


setup(
    name='IMUCapture',

    # Versions should comply with PEP440.  For a discussion on single-sourcing
    # the version across setup.py and the project code, see
    # https://packaging.python.org/en/latest/single_source_version.html

    version = Global_data.VERSION,

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

        'Intended Audience :: Biologists',
        'Topic :: Kinematics collection :: IMU tracking',

        'License :: OSI Approved :: GPLv3 License',

        # Python versions supported
        'Programming Language :: Python :: 3.4',
        'Programming Language :: Python :: 3.5',
    ],

    # What does your project relate to?
    keywords='IMU, fish, kinematics',

    # specify packages
    packages=[package_name],


    # run-time dependencies here.  These will be installed by pip when
    # project is installed. For an analysis of "install_requires" vs pip's
    # requirements files see:
    # https://packaging.python.org/en/latest/requirements.html

    install_requires=['h5py>=2.7.0', 
                      'pyqtgraph>=0.10.0',
                      'pyserial>=3.3',
                      'PyQt5 >= 5.8.2, < 5.11.2', # riverbank's 5.11.2 wheel is broken
                      'pyquaternion>=0.9.2',
                     ],


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
