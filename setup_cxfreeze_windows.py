from cx_Freeze import setup, Executable

# Dependencies are automatically detected, but it might need
# fine tuning.
buildOptions = dict(
    packages = [],
    excludes = [],
    includes = ['numpy.core._methods',
                'numpy.lib.format',
                'pyqtgraph.debug',
                'pyqtgraph.ThreadsafeTimer',
                'timeit',
               ],
    include_files = ['imucapture\\VERSION']
)
    
base = 'Console'
#import sys
#base = 'Win32GUI' if sys.platform=='win32' else None

executables = [
    Executable('imucapture\\__main__.py', base=base, targetName = 'imu_capture.exe')
]


setup(options = dict(build_exe = buildOptions),
      executables = executables)
