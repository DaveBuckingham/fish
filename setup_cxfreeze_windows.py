from cx_Freeze import setup, Executable

VERSION = '0.1'

buildOptions = dict(
    packages = [],
    excludes = [],
    includes = ['numpy.core._methods',
                'numpy.lib.format',
                'pyqtgraph.debug',
                'pyqtgraph.ThreadsafeTimer',
                'timeit',
               ],
)
    
base = 'Console'

executables = [
    Executable('imucapture\\__main__.py', base=base, targetName = 'imu_capture_' + VERSION + '.exe')
]


setup(options = dict(build_exe = buildOptions),
      executables = executables)
