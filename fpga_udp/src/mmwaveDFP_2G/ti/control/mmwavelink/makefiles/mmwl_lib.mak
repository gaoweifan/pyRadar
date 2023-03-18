###################################################################################
#   mmWaveLink Driver Makefile
###################################################################################

###################################################################################
# Setup the VPATH:
###################################################################################
vpath %.c ./src

###################################################################################
# Driver Source Files:
###################################################################################
MMWAVELINK_DRV_SOURCES = rl_controller.c rl_device.c rl_driver.c rl_monitoring.c rl_sensor.c

###################################################################################
# Driver Source Files:
###################################################################################
MMWAVELINK_R4F_DRV_LIB_OBJECTS  = $(addprefix $(MMWAVE_BUILD_PLATFORM_OBJDIR)/,  $(MMWAVELINK_DRV_SOURCES:.c=.$(R4F_OBJ_EXT)))
MMWAVELINK_C674_DRV_LIB_OBJECTS = $(addprefix $(MMWAVE_BUILD_PLATFORM_OBJDIR)/,  $(MMWAVELINK_DRV_SOURCES:.c=.$(C674_OBJ_EXT)))
MMWAVELINK_R5F_DRV_LIB_OBJECTS  = $(addprefix $(MMWAVE_BUILD_PLATFORM_OBJDIR)/,  $(MMWAVELINK_DRV_SOURCES:.c=.$(R5F_OBJ_EXT)))
MMWAVELINK_C66_DRV_LIB_OBJECTS  = $(addprefix $(MMWAVE_BUILD_PLATFORM_OBJDIR)/,  $(MMWAVELINK_DRV_SOURCES:.c=.$(C66_OBJ_EXT)))
MMWAVELINK_M4_DRV_LIB_OBJECTS   = $(addprefix $(MMWAVE_BUILD_PLATFORM_OBJDIR)/,  $(MMWAVELINK_DRV_SOURCES:.c=.$(M4_OBJ_EXT)))

###################################################################################
# Driver Dependency:
###################################################################################
MMWAVELINK_R4F_DRV_DEPENDS  = $(addprefix $(MMWAVE_BUILD_PLATFORM_OBJDIR)/, $(MMWAVELINK_DRV_SOURCES:.c=.$(R4F_DEP_EXT)))
MMWAVELINK_R5F_DRV_DEPENDS  = $(addprefix $(MMWAVE_BUILD_PLATFORM_OBJDIR)/, $(MMWAVELINK_DRV_SOURCES:.c=.$(R5F_DEP_EXT)))
MMWAVELINK_C674_DRV_DEPENDS = $(addprefix $(MMWAVE_BUILD_PLATFORM_OBJDIR)/, $(MMWAVELINK_DRV_SOURCES:.c=.$(C674_DEP_EXT)))
MMWAVELINK_C66_DRV_DEPENDS  = $(addprefix $(MMWAVE_BUILD_PLATFORM_OBJDIR)/, $(MMWAVELINK_DRV_SOURCES:.c=.$(C66_DEP_EXT)))
MMWAVELINK_M4_DRV_DEPENDS   = $(addprefix $(MMWAVE_BUILD_PLATFORM_OBJDIR)/, $(MMWAVELINK_DRV_SOURCES:.c=.$(M4_DEP_EXT)))

###################################################################################
# Driver Library Names:
###################################################################################
MMWAVELINK_R4F_DRV_LIB  = ./lib/mmwavelink_r4f.$(R4F_LIB_EXT)
MMWAVELINK_R5F_DRV_LIB  = ./lib/mmwavelink_r5f.$(R5F_LIB_EXT)
MMWAVELINK_C674_DRV_LIB = ./lib/mmwavelink_c674.$(C674_LIB_EXT)
MMWAVELINK_C66_DRV_LIB  = ./lib/mmwavelink_c66.$(C66_LIB_EXT)
MMWAVELINK_M4_DRV_LIB   = ./lib/mmwavelink_m4.$(M4_LIB_EXT)

###################################################################################
# Build the mmWaveLink Driver Libraries for all platforms
###################################################################################

mmWaveLinkDrvR4F: buildDirectories $(MMWAVELINK_R4F_DRV_LIB_OBJECTS)
	echo "Archiving $@"
	$(R4F_AR) $(R4F_AR_OPTS) $(MMWAVELINK_R4F_DRV_LIB)  $(MMWAVELINK_R4F_DRV_LIB_OBJECTS)

mmWaveLinkDrvR5F: buildDirectories $(MMWAVELINK_R5F_DRV_LIB_OBJECTS)
	echo "Archiving $@"
	$(R5F_AR) $(R5F_AR_OPTS) $(MMWAVELINK_R5F_DRV_LIB)  $(MMWAVELINK_R5F_DRV_LIB_OBJECTS)

mmWaveLinkDrvM4: buildDirectories $(MMWAVELINK_M4_DRV_LIB_OBJECTS)
	echo "Archiving $@"
	$(M4_AR) $(M4_AR_OPTS) $(MMWAVELINK_M4_DRV_LIB) $(MMWAVELINK_M4_DRV_LIB_OBJECTS)

mmWaveLinkDrvC66: buildDirectories $(MMWAVELINK_C66_DRV_LIB_OBJECTS)
	echo "Archiving $@"
	$(C66_AR) $(C66_AR_OPTS) $(MMWAVELINK_C66_DRV_LIB) $(MMWAVELINK_C66_DRV_LIB_OBJECTS)

mmWaveLinkDrvC674: buildDirectories $(MMWAVELINK_C674_DRV_LIB_OBJECTS)
	echo "Archiving $@"
	$(C674_AR) $(C674_AR_OPTS) $(MMWAVELINK_C674_DRV_LIB) $(MMWAVELINK_C674_DRV_LIB_OBJECTS)

###################################################################################
# Clean the mmWaveLink Driver Libraries for all platforms
###################################################################################

mmWaveLinkDrvCleanR4F:
	@echo 'Cleaning the mmWaveLink Driver Library Objects for R4F'
	@$(DEL) $(MMWAVELINK_R4F_DRV_LIB_OBJECTS) $(MMWAVELINK_R4F_DRV_LIB) $(MMWAVELINK_R4F_DRV_DEPENDS)

mmWaveLinkDrvCleanR5F:
	@echo 'Cleaning the mmWaveLink Driver Library Objects for R5F'
	@$(DEL) $(MMWAVELINK_R5F_DRV_LIB_OBJECTS) $(MMWAVELINK_R5F_DRV_LIB) $(MMWAVELINK_R5F_DRV_DEPENDS)

mmWaveLinkDrvCleanM4:
	@echo 'Cleaning the mmWaveLink Driver Library Objects for M4'
	@$(DEL) $(MMWAVELINK_M4_DRV_LIB_OBJECTS) $(MMWAVELINK_M4_DRV_LIB) $(MMWAVELINK_M4_DRV_DEPENDS)

mmWaveLinkDrvCleanC66:
	@echo 'Cleaning the mmWaveLink Driver Library Objects for C66'
	@$(DEL) $(MMWAVELINK_C66_DRV_LIB_OBJECTS) $(MMWAVELINK_C66_DRV_LIB) $(MMWAVELINK_C66_DRV_DEPENDS)

mmWaveLinkDrvCleanC674:
	@echo 'Cleaning the mmWaveLink Driver Library Objects for C674'
	@$(DEL) $(MMWAVELINK_C674_DRV_LIB_OBJECTS) $(MMWAVELINK_C674_DRV_LIB) $(MMWAVELINK_C674_DRV_DEPENDS)

###################################################################################
# Dependency handling
###################################################################################
-include $(MMWAVELINK_R4F_DRV_DEPENDS)
-include $(MMWAVELINK_R5F_DRV_DEPENDS)
-include $(MMWAVELINK_C674_DRV_DEPENDS)
-include $(MMWAVELINK_C66_DRV_DEPENDS)
-include $(MMWAVELINK_M4_DRV_DEPENDS)
