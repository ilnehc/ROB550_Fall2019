SRC_PATH 	:= $(shell pwd)/../.
BIN_PATH	:= $(SRC_PATH)/bin/.
COMMON_LIB_PATH := $(SRC_PATH)/optitrack/common/lib/.

CXX		:= g++

CFLAGS_SHARED	:= -g -DFILE_OFFSET_BITS=64 -D_LARGEFILE_SOURCE -D_REENTRANT \
			-Wall -Wno-unused-parameter -Wno-deprecated-declarations \
			-Wno-unused-result \
			-pthread -fPIC -I $(SRC_PATH)	

CFLAGS_STD	:= -std=gnu99 -Wno-format-zero-length $(CFLAGS_SHARED)
CXXFLAGS_STD 	:= -std=c++0x $(CFLAGS_SHARED) 

LDFLAGS_STD	:= -lm -lpthread
LDFLAGS_COMMON	:= -L -lcommon $(COMMON_LIB_PATH)/libcommon.a

CXXFLAGS = $(CXXFLAGS_STD) \
	-O3
	
LDFLAGS = $(LDFLAGS_STD) \
	$(LDFLAGS_COMMON) \

SOURCES		:= $(wildcard *.c)
CPP_SOURCES		:= $(wildcard *.cpp)
INCLUDES	:= $(wildcard *.h)
OBJECTS		:= $(SOURCES:$%.c=$%.o)
CPP_OBJECTS		:= $(CPP_SOURCES:$%.cpp=$%.o)

BIN_OPTITRACK_DRIVER=$(BIN_PATH)/optitrack_driver
BIN_RECEIVE_SERIAL_OPTI=$(BIN_PATH)/receive_serial_opti

all: $(BIN_RECEIVE_SERIAL_OPTI) $(BIN_OPTITRACK_DRIVER)

# Make optitrack executables
$(BIN_OPTITRACK_DRIVER): optitrack.o optitrack_driver.o
	@$(CXX) -o $@ $^ $(CXXFLAGS) $(LDFLAGS)

$(BIN_RECEIVE_SERIAL_OPTI): receive_serial_opti.o
	@$(CXX) -o $@ $^ $(CXXFLAGS) $(LDFLAGS)

# Clean target
clean:
	@rm -f *~ *.o $(BIN_OPTITRACK_DRIVER) $(BIN_RECEIVE_SERIAL_OPTI) *.a
	@echo "../bin/receive_serial_opti Clean Complete"
	@echo "../bin/optitrack_driver Clean Complete"
