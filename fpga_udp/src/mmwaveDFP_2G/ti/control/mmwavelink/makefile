###################################################################################
# mmWaveLink makefile
###################################################################################
include ./makefiles/mmwl_build_common.mak
include ./makefiles/mmwl_lib.mak

.PHONY: all clean help

# Build mmWaveLink driver for all platforms
all: mmWaveLinkDrvR4F mmWaveLinkDrvR5F mmWaveLinkDrvM4 mmWaveLinkDrvC66 mmWaveLinkDrvC674

# Build mmWaveLink driver for R4F platform
r4f: mmWaveLinkDrvR4F

# Build mmWaveLink driver for R5F platform
r5f: mmWaveLinkDrvR5F

# Build mmWaveLink driver for M4 platform
m4: mmWaveLinkDrvM4

# Build mmWaveLink driver for C66 platform
c66: mmWaveLinkDrvC66

# Build mmWaveLink driver for C674 platform
c674: mmWaveLinkDrvC674

# Cleans mmWaveLink objects for all platforms
clean: mmWaveLinkDrvCleanR4F mmWaveLinkDrvCleanR5F mmWaveLinkDrvCleanM4 mmWaveLinkDrvCleanC66 mmWaveLinkDrvCleanC674

# Cleans mmWaveLink objects for R4F platform
r4fClean: mmWaveLinkDrvCleanR4F

# Cleans mmWaveLink objects for R5F platform
r5fClean: mmWaveLinkDrvCleanR5F

# Cleans mmWaveLink objects for M4 platform
m4Clean: mmWaveLinkDrvCleanM4

# Cleans mmWaveLink objects for C66 platform
c66Clean: mmWaveLinkDrvCleanC66

# Cleans mmWaveLink objects for C674 platform
c674Clean: mmWaveLinkDrvCleanC674

# Help: This displays the MAKEFILE Usage.
help:
	@echo '*******************************************************************************************************'
	@echo '* Makefile Targets for the mmWaveLink *'
	@echo '* all                                          : Build the mmWaveLink library for all platforms'
	@echo '* r4f/r5f/m4/c66/c674                          : Build the mmWaveLink library for the specified platform'
	@echo '* clean                                        : Clean the mmWaveLink objects for all platforms'
	@echo '* r4fClean/r5fClean/m4Clean/c66Clean/c674Clean : Clean the mmWaveLink objects for the specified platform'
	@echo '*******************************************************************************************************'

