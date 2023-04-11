##################################################################################
# GTRACK makefile
##################################################################################
include ../../common/mmwave_sdk.mak
include ./gtracklib.mak

##################################################################################
# SOC Specific Test Targets
##################################################################################
include ./test/usecases/mss/gtrackTestMss.mak
include ./test/usecases/dss/gtrackTestDss.mak

###################################################################################
# Standard Targets which need to be implemented by each mmWave SDK module. This
# plugs into the release scripts.
###################################################################################
.PHONY: all clean lib libClean test testClean help
.PHONY: gtracklib2d gtracklib3d
.PHONY: gtracktest2d gtracktest3d gtracktestMss gtracktestDss gtrackTestMss2d gtrackTestMss3d gtrackTestDss2d gtrackTestDss3d

##################################################################################
# Build/Clean the driver
##################################################################################

# This builds GTRACK Libraries
lib:
	$(MAKE) -s MMWAVE_SDK_LIB_BUILD_OPTION=2D gtracklib
	$(MAKE) -s MMWAVE_SDK_LIB_BUILD_OPTION=3D gtracklib

# This builds the 2D GTRACK Library
gtracklib2d:
	$(MAKE) -s MMWAVE_SDK_LIB_BUILD_OPTION=2D gtracklib

# This builds the 3D GTRACK Library
gtracklib3d:
	$(MAKE) -s MMWAVE_SDK_LIB_BUILD_OPTION=3D gtracklib
		
# This cleans GTRACK Libraries
libClean:
	$(MAKE) -s MMWAVE_SDK_LIB_BUILD_OPTION=2D gtracklibClean
	$(MAKE) -s MMWAVE_SDK_LIB_BUILD_OPTION=3D gtracklibClean

# This cleans the 2D GTRACK Library
gtracklib2dClean:
	$(MAKE) -s MMWAVE_SDK_LIB_BUILD_OPTION=2D gtracklibClean

# This cleans the 3D GTRACK Library
gtracklib3dClean:
	$(MAKE) -s MMWAVE_SDK_LIB_BUILD_OPTION=3D gtracklibClean
	
##################################################################################
# Test targets which are device specific
# AM273X, AWR2943 and AWR2944: Build the MSS and DSS Unit Test
##################################################################################
allTest:
	$(MAKE) -s MMWAVE_SDK_LIB_BUILD_OPTION=2D gtrackTestDss
	$(MAKE) -s MMWAVE_SDK_LIB_BUILD_OPTION=3D gtrackTestDss

	$(MAKE) -s MMWAVE_SDK_LIB_BUILD_OPTION=2D gtrackTestMss
	$(MAKE) -s MMWAVE_SDK_LIB_BUILD_OPTION=3D gtrackTestMss

allTestClean:
	$(MAKE) -s MMWAVE_SDK_LIB_BUILD_OPTION=2D gtrackTestMssClean
	$(MAKE) -s MMWAVE_SDK_LIB_BUILD_OPTION=3D gtrackTestMssClean

	$(MAKE) -s MMWAVE_SDK_LIB_BUILD_OPTION=2D gtrackTestDssClean
	$(MAKE) -s MMWAVE_SDK_LIB_BUILD_OPTION=3D gtrackTestDssClean

gtracktest2d:
	$(MAKE) -s MMWAVE_SDK_LIB_BUILD_OPTION=2D gtrackTestMss
	$(MAKE) -s MMWAVE_SDK_LIB_BUILD_OPTION=2D gtrackTestDss

gtracktest3d:
	$(MAKE) -s MMWAVE_SDK_LIB_BUILD_OPTION=3D gtrackTestMss
	$(MAKE) -s MMWAVE_SDK_LIB_BUILD_OPTION=3D gtrackTestDss

gtracktestMss:
	$(MAKE) -s MMWAVE_SDK_LIB_BUILD_OPTION=2D gtrackTestMss
	$(MAKE) -s MMWAVE_SDK_LIB_BUILD_OPTION=3D gtrackTestMss

gtracktestDss:
	$(MAKE) -s MMWAVE_SDK_LIB_BUILD_OPTION=2D gtrackTestDss
	$(MAKE) -s MMWAVE_SDK_LIB_BUILD_OPTION=3D gtrackTestDss

gtracktestMss2d:
	$(MAKE) -s MMWAVE_SDK_LIB_BUILD_OPTION=2D gtrackTestMss

gtracktestMss3d:
	$(MAKE) -s MMWAVE_SDK_LIB_BUILD_OPTION=3D gtrackTestMss

gtracktestDss2d:
	$(MAKE) -s MMWAVE_SDK_LIB_BUILD_OPTION=2D gtrackTestDss

gtracktestDss3d:
	$(MAKE) -s MMWAVE_SDK_LIB_BUILD_OPTION=3D gtrackTestDss


msssyscfg: test/usecases/mss/test_$(PLATFORM_DEVICE_TYPE).syscfg
	@echo Generating SysConfig files ...
	$(SYSCFG_NODE) $(SYSCFG_CLI_PATH)/dist/cli.js --product $(SYSCFG_SDKPRODUCT) --context r5fss0-0 --part Default --package $(PACKAGE_TYPE) --output test/usecases/mss/mssgenerated/ test/usecases/mss/test_$(PLATFORM_DEVICE_TYPE).syscfg

dsssyscfg: test/usecases/dss/test_$(PLATFORM_DEVICE_TYPE).syscfg
	@echo Generating SysConfig files ...
	$(SYSCFG_NODE) $(SYSCFG_CLI_PATH)/dist/cli.js --product $(SYSCFG_SDKPRODUCT) --context c66ss0 --part Default --package $(PACKAGE_TYPE) --output test/usecases/dss/dssgenerated/ test/usecases/dss/test_$(PLATFORM_DEVICE_TYPE).syscfg

# syscfg: This cleans syscfg files
syscfgClean: 
	@echo 'Cleaning the syscfg files'
	@$(DEL) test/usecases/mss/mssgenerated
	@$(DEL) test/usecases/dss/dssgenerated

syscfg: msssyscfg dsssyscfg

test: syscfg allTest

testClean: syscfgClean allTestClean

# Clean: This cleans all the objects
clean: libClean testClean

# Build everything
all: lib test

# Help: This displays the MAKEFILE Usage.
help:
	@echo '****************************************************************************************'
	@echo '* Makefile Targets for GTRACK '
	@echo 'all            -> Build gtrack lib and all functional unit tests'
	@echo 'clean          -> Clean out all the gtrack objects'
	@echo 'lib            -> Build the gtrack algorithm Library only'
	@echo 'libClean       -> Clean the gtrack algorithm Library only'
	@echo 'test           -> Builds all the gtrack unit tests for the SOC'
	@echo 'testClean      -> Cleans all the gtrack unit tests for the SOC'
	@echo 'syscfg         -> Generates syscfg files for gtrack unit tests'
	@echo 'syscfgClean    -> Cleans syscfg files of gtrack unit tests'
	@echo '****************************************************************************************'
