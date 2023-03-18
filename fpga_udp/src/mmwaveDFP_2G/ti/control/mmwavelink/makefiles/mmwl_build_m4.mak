###################################################################################
# Makefile for M4 platform
###################################################################################

###################################################################################
# Tool Definitions:
###################################################################################
M4_CC    = $(ARM_CODEGEN_INSTALL_PATH)/bin/armcl
M4_LD    = $(ARM_CODEGEN_INSTALL_PATH)/bin/armcl
M4_AR    = $(ARM_CODEGEN_INSTALL_PATH)/bin/armar

###################################################################################
# M4 Tools Flag definitions:
###################################################################################

M4_CFLAGS_ENUM_TYPE   = --enum_type=int

# Setup the Include paths for the M4 Builds:
M4_INCLUDE = -i$(MMWAVE_DFP_INSTALL_PATH) -i$(ARM_CODEGEN_INSTALL_PATH)/include $(STD_INCL)

# Compiler flags used for the M4 Builds:
M4_CFLAGS   = -mv7M4 --code_state=16 --float_support=vfplib --abi=eabi -me --define=_LITTLE_ENDIAN   \
               $(M4_INCLUDE) -g -O3 -display_error_number --diag_warning=225  --little_endian        \
              --diag_wrap=off --preproc_with_compile --gen_func_subsections                          \
              --emit_warnings_as_errors $(M4_CFLAGS_ENUM_TYPE)

# Linker flags used for the M4 Builds:
M4_LDFLAGS  = -mv7M4 --code_state=16 --float_support=vfplib --abi=eabi -me -g         \
              --display_error_number --diag_warning=225 --diag_wrap=off -z              \
              --reread_libs --warn_sections --rom_model --unused_section_elimination    \
              -i$(ARM_CODEGEN_INSTALL_PATH)/lib --reread_libs --emit_warnings_as_errors

# all options used to enable build warnings as errors
EMIT_WARNINGS_AS_ERRORS_OPTIONS := --emit_warnings_as_errors -pdew

# if MMWAVE_DISABLE_WARNINGS_AS_ERRORS is 1 then remove the emit_warnings_as_errors flag
ifneq ("$(MMWAVE_DISABLE_WARNINGS_AS_ERRORS)", "")
    ifeq ("$(MMWAVE_DISABLE_WARNINGS_AS_ERRORS)","1")
        M4_CFLAGS := $(filter-out $(EMIT_WARNINGS_AS_ERRORS_OPTIONS),$(M4_CFLAGS))
        M4_LDFLAGS := $(filter-out $(EMIT_WARNINGS_AS_ERRORS_OPTIONS),$(M4_LDFLAGS))
    endif
endif  

# Archiver options:
M4_AR_OPTS = r

# File extension to use for M4 library file
M4_LIB_EXT = lib

# File extension to use for M4 Objects
M4_OBJ_EXT = oem4

# File extension to use for M4 Dependency
M4_DEP_EXT = d
