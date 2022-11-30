##
## 作者：ayowin
## 源自：https://github.com/ayowin/reusable_makefile
## 描述：
##		仅作开源技术分享，可用于任何用途，但不做任何技术支持，请知悉
##

TARGET = $(notdir $(PWD))
CC = g++
# Tips: if you want to build release target , it is suggested to remove '-g' flag.
CFLAGS = -std=c++11 -g -lpthread
INCLUDEPATH = $(addprefix -I,$(ALL_DIRS))
LIBS = 
BUILD_PATH = build
EXCLUDE_DIRS = ./.git ./.svn
EXCLUDE_DIRS := $(foreach dir,$(EXCLUDE_DIRS),-path $(dir) -prune -o)
ALL_DIRS := $(shell find . -maxdepth 5 $(EXCLUDE_DIRS) -type d -print)
SOURCE_FILES = $(foreach dir,$(ALL_DIRS),$(wildcard $(dir)/*.cpp))
OBJECT_FILES = $(notdir $(patsubst %.cpp,%.o, $(SOURCE_FILES)))
HEDAER_DEPENDS = $(patsubst %o,%d,$(OBJECT_FILES))
HEDAER_DEPENDS := $(addprefix $(BUILD_PATH)/,$(HEDAER_DEPENDS))

# VPATH update
VPATH += $(ALL_DIRS)

# .PHONY declare: we suggest goals declared .PHONY which they have no dependencies
# .PHONY: pre clean run

# all
all: pre compile link 

# link
link: $(TARGET)
$(TARGET): compile
	$(CC) $(BUILD_PATH)/*.o -o $(BUILD_PATH)/$(TARGET) $(LIBS) $(CFLAGS)

# compile
compile: $(OBJECT_FILES) $(HEDAER_DEPENDS)
$(OBJECT_FILES): %.o : %.cpp %.d
	$(CC) -c $< -o $(BUILD_PATH)/$@ $(INCLUDEPATH) $(LIBS) $(CFLAGS)
	
# header files modified defection for compile
$(HEDAER_DEPENDS): pre
-include $(HEDAER_DEPENDS)
%.d: %.cpp
	@set -e;
	@rm -f $(BUILD_PATH)/$@;
	@$(CC) $(CFLAGS) $(INCLUDEPATH) -MM $< > $(BUILD_PATH)/$@.$$$$; \
		sed 's,\($*\)\.o[ :]*,\1.o $@ : ,g' < $(BUILD_PATH)/$@.$$$$ > $(BUILD_PATH)/$@; \
		rm -f $(BUILD_PATH)/$@.$$$$

# pre
pre:
ifneq ($(BUILD_PATH), $(wildcard $(BUILD_PATH)))
	@mkdir -p $(BUILD_PATH)
endif

# run
run:
	sudo ./$(BUILD_PATH)/$(TARGET)

# clean
clean:
	rm -rf $(BUILD_PATH)
	@echo "Clean success."
