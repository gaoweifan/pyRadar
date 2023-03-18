###################################################################################
# mmWaveLink common makefile
###################################################################################

###################################################################################
# Tool Definitions:
###################################################################################
XDC      = $(XDC_INSTALL_PATH)/xdc
XS       = $(XDC_INSTALL_PATH)/xs
MMWAVE_BUILD_PLATFORM_OBJDIR         = ./makefiles/obj_mmWaveLink

###################################################################################
# OS specific definitions:
###################################################################################
ifeq ($(OS),Windows_NT)
DEL      = $(XDC_INSTALL_PATH)/bin/rm -Rf
MKDIR    = $(XDC_INSTALL_PATH)/bin/mkdir
MAKE     = gmake
else
DEL      = rm -Rf
MKDIR    = mkdir
MAKE     = make
endif

###################################################################################
# Platform specific definitions:
###################################################################################
ifneq (,$(findstring r4f,$(MMWAVE_BUILD_PLATFORM)))
include ./makefiles/mmwl_build_r4f.mak
endif
ifneq (,$(findstring r5f,$(MMWAVE_BUILD_PLATFORM)))
include ./makefiles/mmwl_build_r5f.mak
endif
ifneq (,$(findstring m4,$(MMWAVE_BUILD_PLATFORM)))
include ./makefiles/mmwl_build_m4.mak
endif
ifneq (,$(findstring c66,$(MMWAVE_BUILD_PLATFORM)))
include ./makefiles/mmwl_build_c66.mak
endif
ifneq (,$(findstring c674,$(MMWAVE_BUILD_PLATFORM)))
include ./makefiles/mmwl_build_c674.mak
endif

###################################################################################
# Build the object directory
###################################################################################
buildDirectories:
	@$(MKDIR) -p $(MMWAVE_BUILD_PLATFORM_OBJDIR)

###################################################################################
# Build Suffix:
###################################################################################
ifneq (,$(findstring r4f,$(MMWAVE_BUILD_PLATFORM)))
$(MMWAVE_BUILD_PLATFORM_OBJDIR)/%.$(R4F_OBJ_EXT): %.c
	@echo '[R4 Device/Type] Building file: $<'
	@$(R4F_CC) -c $(R4F_CFLAGS) -ppd=$(MMWAVE_BUILD_PLATFORM_OBJDIR)/"$(basename $(<F)).$(R4F_DEP_EXT)"  $< --output_file $@
$(MMWAVE_BUILD_PLATFORM_OBJDIR)/%.$(R4F_ASM_OBJ_EXT): %.asm
	@echo '[R4 Device/Type] Building file: $<'
	@$(R4F_CC) -c $(R4F_CFLAGS)  --asm_dependency="$<.d"  $< --output_file $@
	@$(R4F_CC) -c $(R4F_CFLAGS) $< --output_file $@
endif

ifneq (,$(findstring r5f,$(MMWAVE_BUILD_PLATFORM)))
$(MMWAVE_BUILD_PLATFORM_OBJDIR)/%.$(R5F_OBJ_EXT): %.c
	@echo '[R5 Device/Type] Building file: $<'
	@$(R5F_CC) -c $(R5F_CFLAGS) -ppd=$(MMWAVE_BUILD_PLATFORM_OBJDIR)/"$(basename $(<F)).$(R5F_DEP_EXT)"  $< --output_file $@ 
$(MMWAVE_BUILD_PLATFORM_OBJDIR)/%.$(R5F_ASM_OBJ_EXT): %.asm
	@echo '[R5 Device/Type] Building file: $<'
	@$(R5F_CC) -c $(R5F_CFLAGS)  --asm_dependency="$<.d"  $< --output_file $@
	@$(R5F_CC) -c $(R5F_CFLAGS) $< --output_file $@
endif

ifneq (,$(findstring m4,$(MMWAVE_BUILD_PLATFORM)))
$(MMWAVE_BUILD_PLATFORM_OBJDIR)/%.$(M4_OBJ_EXT): %.c
	@echo '[M4 Device/Type] Building file: $<'
	@$(M4_CC) -c $(M4_CFLAGS) -ppd=$(MMWAVE_BUILD_PLATFORM_OBJDIR)/"$(basename $(<F)).$(M4_DEP_EXT)"  $< --output_file $@
endif

ifneq (,$(findstring c674,$(MMWAVE_BUILD_PLATFORM)))
$(MMWAVE_BUILD_PLATFORM_OBJDIR)/%.$(C674_OBJ_EXT): %.c
	@echo '[C674 Device/Type] Building file: $<'
	@$(C674_CC) -c $(C674_CFLAGS) -ppd=$(MMWAVE_BUILD_PLATFORM_OBJDIR)/"$(basename $(<F)).$(C674_DEP_EXT)" "$<" --output_file $@
endif

ifneq (,$(findstring c66,$(MMWAVE_BUILD_PLATFORM)))
$(MMWAVE_BUILD_PLATFORM_OBJDIR)/%.$(C66_OBJ_EXT): %.c
	@echo '[C66 Device/Type] Building file: $<'
	@$(C66_CC) -c $(C66_CFLAGS) -ppd=$(MMWAVE_BUILD_PLATFORM_OBJDIR)/"$(basename $(<F)).$(C66_DEP_EXT)" "$<" --output_file $@
endif
