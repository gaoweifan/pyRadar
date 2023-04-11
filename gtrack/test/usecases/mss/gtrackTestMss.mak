###################################################################################
# GTRACK Usecase Unit Test on MSS Makefile
###################################################################################
.PHONY: gtrackTestMss gtrackTestMssClean

###################################################################################
# Setup the VPATH:
###################################################################################
vpath %.c src
vpath %.c test/usecases/mss
vpath %.asm test/usecases/mss
vpath %.c test/common

###################################################################################
# The GTRACK Unit test requires additional libraries
###################################################################################
GTRACK_USECASE_MSS_TEST_STD_LIBS = $(R5F_COMMON_STD_LIB) \
								  -llibgtrack$(MMWAVE_SDK_LIB_BUILD_OPTION)_$(MMWAVE_SDK_DEVICE_TYPE).$(R5F_LIB_EXT) \
								  -llibtestlogger_$(MMWAVE_SDK_DEVICE_TYPE).$(R5F_LIB_EXT)						  
GTRACK_USECASE_MSS_TEST_LOC_LIBS = $(R5F_COMMON_LOC_LIB) \
								  -Wl,-i${MMWAVE_SDK_INSTALL_PATH}/ti/utils/testlogger/lib \
								  -Wl,-ilib \

###################################################################################
# Unit Test Files
###################################################################################
GTRACK_USECASE_MSS_TEST_CMD		= $(MMWAVE_SDK_INSTALL_PATH)/ti/platform/$(MMWAVE_SDK_DEVICE_TYPE)
GTRACK_USECASE_MSS_TEST_MAP		= test/usecases/mss/$(MMWAVE_SDK_DEVICE_TYPE)_gtrack$(MMWAVE_SDK_LIB_BUILD_OPTION)_usecase_mss.map
GTRACK_USECASE_MSS_TEST_OUT		= test/usecases/mss/$(MMWAVE_SDK_DEVICE_TYPE)_gtrack$(MMWAVE_SDK_LIB_BUILD_OPTION)_usecase_mss.$(R5F_EXE_EXT)
GTRACK_USECASE_MSS_TEST_BIN		= test/usecases/mss/$(MMWAVE_SDK_DEVICE_TYPE)_gtrack$(MMWAVE_SDK_LIB_BUILD_OPTION)_usecase_mss.bin
GTRACK_USECASE_MSS_TEST_APP_CMD	= test/usecases/mss/mss_gtrack_linker.cmd
GTRACK_USECASE_MSS_TEST_SOURCES	= main_mss.c	\
								  gtrackApp.c		\
								  gtrackAlloc.c		\
								  gtrackLog.c

GTRACK_USECASE_MSS_TEST_SOURCES_GEN = ti_board_config.c	\
									  ti_board_open_close.c	\
									  ti_dpl_config.c	\
									  ti_drivers_config.c	\
									  ti_pinmux_config.c	\
									  ti_power_clock_config.c	\
									  ti_drivers_open_close.c

# Get the list of C and assembly files
GTRACK_USECASE_MSS_TEST_C_SOURCES   = $(filter %.c,   $(GTRACK_USECASE_MSS_TEST_SOURCES))
GTRACK_USECASE_MSS_TEST_ASM_SOURCES = $(filter %.asm, $(GTRACK_USECASE_MSS_TEST_SOURCES))

GTRACK_USECASE_MSS_TEST_DEPEND        = $(addprefix $(PLATFORM_OBJDIR)/, $(GTRACK_USECASE_MSS_TEST_C_SOURCES:.c=.$(R5F_DEP_EXT)))
GTRACK_USECASE_MSS_TEST_DEPENDS      += $(addprefix $(PLATFORM_OBJDIR)/, $(GTRACK_USECASE_MSS_TEST_ASM_SOURCES:.asm=.$(R5F_DEP_EXT)))
GTRACK_USECASE_MSS_TEST_DEPENDS      += $(addprefix $(PLATFORM_OBJDIR)/mssgenerated/, $(GTRACK_USECASE_MSS_TEST_SOURCES_GEN:.c=.$(R5F_DEP_EXT)))

GTRACK_USECASE_MSS_TEST_OBJECTS 	 = $(addprefix $(PLATFORM_OBJDIR)/, $(GTRACK_USECASE_MSS_TEST_C_SOURCES:.c=.$(R5F_OBJ_EXT)))
GTRACK_USECASE_MSS_TEST_OBJECTS  	+= $(addprefix $(PLATFORM_OBJDIR)/, $(GTRACK_USECASE_MSS_TEST_ASM_SOURCES:.asm=.o))
GTRACK_USECASE_MSS_TEST_OBJECTS     += $(addprefix $(PLATFORM_OBJDIR)/mssgenerated/, $(GTRACK_USECASE_MSS_TEST_SOURCES_GEN:.c=.$(R5F_OBJ_EXT)))

###################################################################################
# Build Unit Test for 2D Tracker:
###################################################################################
gtrackTestMss: R5F_CFLAGS += -DGTRACK_$(MMWAVE_SDK_LIB_BUILD_OPTION)
gtrackTestMss: buildDirectories mssbuildDirectories $(GTRACK_USECASE_MSS_TEST_OBJECTS) mssbuildDirectories
	$(R5F_LD) $(R5F_LDFLAGS) $(GTRACK_USECASE_MSS_TEST_LOC_LIBS) -Wl,-m=$(GTRACK_USECASE_MSS_TEST_MAP) \
	-o $(GTRACK_USECASE_MSS_TEST_OUT) $(GTRACK_USECASE_MSS_TEST_OBJECTS) $(GTRACK_USECASE_MSS_TEST_STD_LIBS) \
	$(PLATFORM_R5F_LINK_CMD) $(GTRACK_USECASE_MSS_TEST_APP_CMD)
	@echo '******************************************************************************'
	@echo 'Built the GTRACK $(MMWAVE_SDK_LIB_BUILD_OPTION) MSS Unit Test '
	@echo '******************************************************************************'

###################################################################################
# Cleanup Unit Test:
###################################################################################
gtrackTestMssClean:
	@echo 'Cleaning the GTRACK $(MMWAVE_SDK_LIB_BUILD_OPTION) MSS Unit Test objects'
	@$(DEL) $(GTRACK_USECASE_MSS_TEST_OBJECTS) $(GTRACK_USECASE_MSS_TEST_OUT) $(GTRACK_USECASE_MSS_TEST_BIN)
	@$(DEL) $(GTRACK_USECASE_MSS_TEST_MAP) $(GTRACK_USECASE_MSS_TEST_DEPENDS)
	@$(DEL) $(PLATFORM_OBJDIR)

###################################################################################
# Dependency handling
###################################################################################
-include $(GTRACK_USECASE_MSS_TEST_DEPENDS)

