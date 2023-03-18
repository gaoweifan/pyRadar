###############################################################################
#
# Set up tools and build environment variables for mmWaveLink library generation
#
###############################################################################

# Select your build platform(s). Options (case sensitive) are: r4f, r5f, m4, c66, c674
# Multiple platforms can also be selected separated by spaces.
export MMWAVE_BUILD_PLATFORM=r4f r5f m4 c66 c674

export MMWAVE_DFP_TOOLS_INSTALL_PATH=__MMWAVE_DFP_TOOLS_INSTALL_PATH__
 
# Common settings for all platforms
# Path to <mmwave_dfp installation path> folder
export MMWAVE_SDK_INSTALL_PATH=${MMWAVE_DFP_TOOLS_INSTALL_PATH}/mmwave_dfp_02_02_03_00
# TI ARM compiler
export ARM_CODEGEN_INSTALL_PATH=${MMWAVE_DFP_TOOLS_INSTALL_PATH}/ti-cgt-arm_18.12.5.LTS
# TI XDC TOOLS
export XDC_INSTALL_PATH=${MMWAVE_DFP_TOOLS_INSTALL_PATH}/xdctools_3_50_08_24_core
  
# Following only needed for devices with DSP
# TI DSP compiler
export DSP_CODEGEN_INSTALL_PATH=${MMWAVE_DFP_TOOLS_INSTALL_PATH}/ti-cgt-c6000_8.3.3
