from cx_Freeze import setup, Executable

# Dependencies are automatically detected, but it might need
# fine tuning.
#buildOptions = dict(packages = ['h5py', 'numpy', 'pyqtgraph', 'serial', 'six'], excludes = [])
buildOptions = dict(packages = [], excludes = [])

import sys
base = 'Win32GUI' if sys.platform=='win32' else None

executables = [
    Executable('fish/__main__.py', base=base, targetName = 'fishx')
]

setup(name='fishx',
      version = '1.0',
      description = 'awesome',
      options = dict(build_exe = buildOptions),
      executables = executables)
