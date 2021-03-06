from cx_Freeze import setup, Executable
from imucapture.global_data import Global_data

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


setup(name = 'imucapture_sfl' if Global_data.SMART_FILE_LOADING else 'imucapture',
      version = Global_data.VERSION,
      description = 'Record data from IMU',
      options = dict(build_exe = buildOptions),
      executables = executables)
