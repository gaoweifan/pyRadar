###################################################################################
# Makefile for R5F platform
###################################################################################

###################################################################################
# Tool Definitions:
###################################################################################
R5F_CC   = $(ARM_CODEGEN_INSTALL_PATH)/bin/armcl
R5F_LD   = $(ARM_CODEGEN_INSTALL_PATH)/bin/armcl
R5F_AR   = $(ARM_CODEGEN_INSTALL_PATH)/bin/armar

###################################################################################
# R5F Tools Flag definitions:
###################################################################################

R5F_CFLAGS_ENUM_TYPE  = --enum_type=int

# Setup the Include paths for the R5 Builds:
R5F_INCLUDE = -i$(MMWAVE_DFP_INSTALL_PATH) -i$(ARM_CODEGEN_INSTALL_PATH)/include $(STD_INCL)

# Compiler flags used for the R5 Builds:
R5F_CFLAGS  = -mv7R5 --code_state=16 --float_support=VFPv3D16 --abi=eabi -me                   \
              $(R5F_INCLUDE) -g -O3 -display_error_number --diag_warning=225                   \
              --little_endian --diag_wrap=off --preproc_with_compile --gen_func_subsections    \
              --emit_warnings_as_errors $(R5F_CFLAGS_ENUM_TYPE) --define=_LITTLE_ENDIAN        \
              -Dxdc_target_name__=R5Ft -Dxdc_target_types__=ti/targets/arm/elf/std.h

# Linker flags used for the R5 Builds:
R5F_LDFLAGS = -mv7R5 --code_state=16 --float_support=VFPv3D16 --abi=eabi -me -g         \
              --display_error_number --diag_warning=225 --diag_wrap=off -z              \
              --reread_libs --warn_sections --rom_model --unused_section_elimination    \
              -i$(ARM_CODEGEN_INSTALL_PATH)/lib --reread_libs --emit_warnings_as_errors

# all options used to enable build warnings as errors
EMIT_WARNINGS_AS_ERRORS_OPTIONS := --emit_warnings_as_errors -pdew

# if MMWAVE_DISABLE_WARNINGS_AS_ERRORS is 1 then remove the emit_warnings_as_errors flag
ifneq ("$(MMWAVE_DISABLE_WARNINGS_AS_ERRORS)", "")
    ifeq ("$(MMWAVE_DISABLE_WARNINGS_AS_ERRORS)","1")
        R5F_CFLAGS := $(filter-out $(EMIT_WARNINGS_AS_ERRORS_OPTIONS),$(R5F_CFLAGS))
        R5F_LDFLAGS := $(filter-out $(EMIT_WARNINGS_AS_ERRORS_OPTIONS),$(R5F_LDFLAGS))
    endif
endif

# Archiver options:
R5F_AR_OPTS = r

# File extension to use for R5F library file
R5F_LIB_EXT = lib

# File extension to use for R5F Objects
R5F_OBJ_EXT = oer5f

# File extension to use for R5F Asm Objects
R5F_ASM_OBJ_EXT = o5

# File extension to use for R5F Dependency
R5F_DEP_EXT = d
