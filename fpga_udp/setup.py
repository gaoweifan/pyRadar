# Available at setup time due to pyproject.toml
from pybind11.setup_helpers import Pybind11Extension, build_ext
from setuptools import setup
import platform
from glob import glob

__version__ = "1.3.0"

# The main interface is through Pybind11Extension.
# * You can add cxx_std=11/14/17, and then build_ext can be removed.
# * You can set include_pybind11=false to add the include directory yourself,
#   say from a submodule.
#
# Note:
#   Sort input source files if you glob sources to ensure bit-for-bit
#   reproducible builds (https://github.com/pybind/python_example/pull/53)

ext_modules = [
    Pybind11Extension("fpga_udp",
        ["src/main.cpp"]+
        # serial port lib
        ["src/WzSerialportPlus/"+platform.system()+"/WzSerialportPlus.cpp"]+
        # mmwave DFP SingleChip NonOS demo
        sorted(glob("src/mmwaveDFP_2G/ti/example/mmWaveLink_SingleChip_NonOS_Example/*.cpp"))+
        sorted(glob("src/mmwaveDFP_2G/ti/example/mmWaveLink_SingleChip_NonOS_Example/*.c"))+
        # mmwave DFP mmWaveLink Library
        sorted(glob("src/mmwaveDFP_2G/ti/control/mmwavelink/src/*.c"))+
        # mmwave DFP mmWaveLink FTDI Library
        ["src/mmwaveDFP_2G/FTDILib/SourceCode/mmwl_port_ftdi.cpp"]+
        # pevents Library
        ["src/pevents/pevents.cpp"],

                       # serial port lib
        include_dirs = ["src/WzSerialportPlus/"+platform.system()]+
                       # mmwave DFP SingleChip NonOS demo
                       sorted(glob("src/mmwaveDFP_2G/ti/example/mmWaveLink_SingleChip_NonOS_Example/"))+
                       # mmwave DFP mmWaveLink Library
                       ["src/mmwaveDFP_2G/ti/control/mmwavelink/"]+
                       sorted(glob("src/mmwaveDFP_2G/ti/control/mmwavelink/include/"))+
                       # mmwave DFP mmWaveLink FTDI Library
                       ["src/mmwaveDFP_2G/FTDILib/SourceCode/"]+
                       ["src/FTDI_D2XX/"+platform.system()]+
                       # mmwave DFP firmware
                       ["src/mmwaveDFP_2G/firmware/"]+
                       # pevents Library
                       ["src/pevents/"]+
                       # kfifo Library
                       ["src/kfifo/"],
        library_dirs = ["src/FTDI_D2XX/"+platform.system()+"/"+platform.machine()],
        libraries=['ftd2xx'],
        # extra_compile_args=['-g'],
        # Example: passing in the version to the compiled code
        define_macros = [('VERSION_INFO', __version__),('NOMINMAX',1)],
        language='c++',
        cxx_std=17
    ),
]

setup(
    name="fpga_udp",
    version=__version__,
    author="Weifan Gao",
    author_email="gaoweifangao@gmail.com",
    url="https://github.com/gaoweifan/pyRadar",
    description="FPGA UDP reader plugin using pybind11",
    long_description="",
    ext_modules=ext_modules,
    extras_require={"test": "pytest"},
    # Currently, build_ext only provides an optional "highest supported C++
    # level" feature, but in the future it may provide more features.
    # cmdclass={"build_ext": build_ext},
    zip_safe=False,
    python_requires=">=3.7",
)
