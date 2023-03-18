###################################################################################
# Makefile for C674 platform
###################################################################################

###################################################################################
# Tool Definitions:
###################################################################################
C674_CC  = $(DSP_CODEGEN_INSTALL_PATH)/bin/cl6x
C674_AR  = $(DSP_CODEGEN_INSTALL_PATH)/bin/ar6x
C674_LD  = $(DSP_CODEGEN_INSTALL_PATH)/bin/cl6x

###################################################################################
# C674 Tools Flag definitions:
###################################################################################

# Setup the Include paths for C674 Builds:
C674_INCLUDE = -i$(MMWAVE_DFP_INSTALL_PATH) -i$(DSP_CODEGEN_INSTALL_PATH)/include $(STD_INCL)

# Compiler Flags for C674 Builds:
C674_CFLAGS  = -mv6740 --abi=eabi --gcc -g -O3 -mf3 -mo --display_error_number   \
               --diag_warning=225 --diag_wrap=off   --define=_LITTLE_ENDIAN      \
               --preproc_with_compile $(C674_INCLUDE) --emit_warnings_as_errors

# Linker Flags for C674 Builds:
C674_LDFLAGS = -mv6740 --abi=eabi -g --display_error_number           \
               --diag_warning=225 --diag_wrap=off -z --reread_libs --warn_sections --ram_model    \
               -i$(DSP_CODEGEN_INSTALL_PATH)/lib --emit_warnings_as_errors

# Treat "warning #10015-D: output file xyz cannot be loaded and run on a target system" as error
C674_LDFLAGS += --diag_error=10015

# if MMWAVE_DISABLE_WARNINGS_AS_ERRORS is 1 then remove the emit_warnings_as_errors flag
ifneq ("$(MMWAVE_DISABLE_WARNINGS_AS_ERRORS)", "")
    ifeq ("$(MMWAVE_DISABLE_WARNINGS_AS_ERRORS)","1")
        C674_CFLAGS := $(filter-out $(EMIT_WARNINGS_AS_ERRORS_OPTIONS),$(C674_CFLAGS))
        C674_LDFLAGS := $(filter-out $(EMIT_WARNINGS_AS_ERRORS_OPTIONS),$(C674_LDFLAGS))
    endif
endif

# Archiver options:
C674_AR_OPTS = r

# File extension to use for library file
C674_LIB_EXT = lib

# File extension to use for Objects
C674_OBJ_EXT = oe674

# File extension to use for Dependency
C674_DEP_EXT = pp
