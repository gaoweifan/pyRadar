###################################################################################
# Makefile for C66 platform
###################################################################################

###################################################################################
# Tool Definitions:
###################################################################################
C66_CC   = $(DSP_CODEGEN_INSTALL_PATH)/bin/cl6x
C66_AR   = $(DSP_CODEGEN_INSTALL_PATH)/bin/ar6x
C66_LD   = $(DSP_CODEGEN_INSTALL_PATH)/bin/cl6x

###################################################################################
# C66 Tools Flag definitions:
###################################################################################
               
# Setup the Include paths for C66 Builds:
C66_INCLUDE  = -i$(MMWAVE_DFP_INSTALL_PATH) -i$(DSP_CODEGEN_INSTALL_PATH)/include $(STD_INCL)

# Compiler Flags for C66 Builds:
C66_CFLAGS   = -mv6600 --abi=eabi --gcc -g -O3 -mf3 -mo --display_error_number   \
               --diag_warning=225 --diag_wrap=off --define=_LITTLE_ENDIAN        \
               --preproc_with_compile $(C66_INCLUDE) --emit_warnings_as_errors

# Linker Flags for C66 Builds:
C66_LDFLAGS  = -mv6600 --abi=eabi -g --display_error_number           \
               --diag_warning=225 --diag_wrap=off -z --reread_libs --warn_sections --ram_model    \
               -i$(DSP_CODEGEN_INSTALL_PATH)/lib --emit_warnings_as_errors

# Treat "warning #10015-D: output file xyz cannot be loaded and run on a target system" as error
C66_LDFLAGS  += --diag_error=10015

# if MMWAVE_DISABLE_WARNINGS_AS_ERRORS is 1 then remove the emit_warnings_as_errors flag
ifneq ("$(MMWAVE_DISABLE_WARNINGS_AS_ERRORS)", "")
    ifeq ("$(MMWAVE_DISABLE_WARNINGS_AS_ERRORS)","1")
        C66_CFLAGS  := $(filter-out $(EMIT_WARNINGS_AS_ERRORS_OPTIONS),$(C66_CFLAGS))
        C66_LDFLAGS  := $(filter-out $(EMIT_WARNINGS_AS_ERRORS_OPTIONS),$(C66_LDFLAGS))
    endif
endif

# Archiver options:
C66_AR_OPTS = r

# File extension to use for library file
C66_LIB_EXT = lib

# File extension to use for Objects
C66_OBJ_EXT  = oe66

# File extension to use for Dependency
C66_DEP_EXT  = pp
