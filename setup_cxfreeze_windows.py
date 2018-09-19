from cx_Freeze import setup, Executable
from imucapture.global import Global

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
    
executables = [
    Executable(
        'imucapture\\__main__.py',
        base='Console',
        targetName = 'imu_capture.exe',
        shortcutName = 'imucapture',
        shortcutDir = 'DesktopFolder',
        )
]


setup(name='imucapture',
      version = Global.VERSION,
      description = 'Record data from IMU',
      options = dict(build_exe = buildOptions),
      executables = executables)
