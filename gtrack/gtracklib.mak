###################################################################################
# GTRACK Library Makefile
###################################################################################
.PHONY: gtracklib gtracklibClean

###################################################################################
# Setup the VPATH:
###################################################################################
vpath %.c src
vpath %.c platform

###################################################################################
# GTRACK Library Source Files:
###################################################################################
GTRACK_LIB_SOURCES = gtrack_create.c			\
					 gtrack_delete.c			\
					 gtrack_step.c				\
					 gtrack_module.c			\
					 gtrack_unit_create.c		\
					 gtrack_unit_delete.c		\
					 gtrack_unit_get.c			\
					 gtrack_unit_event.c		\
					 gtrack_unit_predict.c		\
					 gtrack_unit_report.c		\
					 gtrack_unit_score.c		\
					 gtrack_unit_start.c		\
					 gtrack_unit_stop.c			\
					 gtrack_unit_update.c		\
					 gtrack_utilities.c			\
					 gtrack_utilities_2d.c		\
					 gtrack_utilities_3d.c		\
					 gtrack_math.c				\
					 gtrack_listlib.c 			

###################################################################################
# Enabling Debug Support
###################################################################################
R5F_CFLAGS  += -DGTRACK_LOG_ENABLED -DGTRACK_ASSERT_ENABLED
C66_CFLAGS  += --define=GTRACK_LOG_ENABLED --define=GTRACK_ASSERT_ENABLED

###################################################################################
# GTRACK Library Source Files:
# - AM273X AWR2943 and AWR2944
#   GTRACK Library is available for the R5 and DSP
###################################################################################
GTRACK_R5F_DRV_LIB_OBJECTS  = $(addprefix $(PLATFORM_OBJDIR)/, $(GTRACK_LIB_SOURCES:.c=.$(R5F_OBJ_EXT)))
GTRACK_C66_DRV_LIB_OBJECTS = $(addprefix $(PLATFORM_OBJDIR)/, $(GTRACK_LIB_SOURCES:.c=.$(C66_OBJ_EXT)))

###################################################################################
# Library Dependency:
###################################################################################
GTRACK_R5F_DRV_DEPENDS  = $(addprefix $(PLATFORM_OBJDIR)/, $(GTRACK_LIB_SOURCES:.c=.$(R5F_DEP_EXT)))
GTRACK_C66_DRV_DEPENDS = $(addprefix $(PLATFORM_OBJDIR)/, $(GTRACK_LIB_SOURCES:.c=.$(C66_DEP_EXT)))

###################################################################################
# GTRACK Library Names:
###################################################################################
GTRACK_R5F_DRV_LIB  = lib/libgtrack$(MMWAVE_SDK_LIB_BUILD_OPTION)_$(MMWAVE_SDK_DEVICE_TYPE).$(R5F_LIB_EXT)
GTRACK_C66_DRV_LIB = lib/libgtrack$(MMWAVE_SDK_LIB_BUILD_OPTION)_$(MMWAVE_SDK_DEVICE_TYPE).$(C66_LIB_EXT)

R5F_CFLAGS += -I../ -I ../include

###################################################################################
# GTRACK Library Build:
###################################################################################
gtracklib: R5F_CFLAGS  += -DGTRACK_$(MMWAVE_SDK_LIB_BUILD_OPTION)
gtracklib: C66_CFLAGS  += --define=GTRACK_$(MMWAVE_SDK_LIB_BUILD_OPTION)
gtracklib: buildDirectories $(GTRACK_R5F_DRV_LIB_OBJECTS) $(GTRACK_C66_DRV_LIB_OBJECTS)
	if [ ! -d "lib" ]; then mkdir lib; fi
	echo "Archiving $@"
	$(R5F_AR) $(R5F_AR_OPTS) $(GTRACK_R5F_DRV_LIB) $(GTRACK_R5F_DRV_LIB_OBJECTS)
	$(C66_AR) $(C66_AR_OPTS) $(GTRACK_C66_DRV_LIB) $(GTRACK_C66_DRV_LIB_OBJECTS)


###################################################################################
# Clean the GTRACK Libraries
###################################################################################
gtracklibClean:
	@echo 'Cleaning the GTRACK Library Objects'
	@$(DEL) $(GTRACK_R5F_DRV_LIB_OBJECTS) $(GTRACK_R5F_DRV_LIB)
	@$(DEL) $(GTRACK_C66_DRV_LIB_OBJECTS) $(GTRACK_C66_DRV_LIB)
	@$(DEL) $(GTRACK_R5F_DRV_DEPENDS) $(GTRACK_C66_DRV_DEPENDS)
	@$(DEL) $(PLATFORM_OBJDIR)

###################################################################################
# Dependency handling
###################################################################################
-include $(GTRACK_R5F_DRV_DEPENDS)
-include $(GTRACK_C66_DRV_DEPENDS)

