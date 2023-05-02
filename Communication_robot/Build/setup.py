from pathlib import Path

from pybind11.setup_helpers import Pybind11Extension, build_ext
from setuptools import setup

jaco_module = Pybind11Extension(
    'jaco2',
    [str(fname) for fname in Path('src').glob('*.cpp')],
    include_dirs=['./lib'],
    extra_compile_args=['-O3', '/utf-8'],
    extra_link_args=['-Wl,-rpath,$ORIGIN/../lib']
)

setup(
    name='jaco2',
    version=0.1,
    author='Daniel Dharampal',
    description='A binding of simple jaco2 SDK commands for python',
    ext_modules=[jaco_module],
    cmdclass={"build_ext": build_ext},
)