from setuptools import setup
from pybind11.setup_helpers import Pybind11Extension, build_ext


ext_modules = [
    Pybind11Extension(
        "fast_math",
        ["fast_math.cpp"],
        cxx_std=17,
    )
]


setup(
    name="radar_swarm_fast_math",
    version="0.1.0",
    description="pybind11 acceleration kernels for RADAR-SWARM",
    ext_modules=ext_modules,
    cmdclass={"build_ext": build_ext},
)
