LDFLAGS_STD	:= -lm -lpthread
LDFLAGS		= $(LDFLAGS_STD)
LIBDEPS		= $(call libdeps, $(LDFLAGS))
LIB_PATH	= $(shell pwd)/lib/

CC		= gcc
CFLAGS_SHARED	= -g -c -Wall -pthread
CFLAGS_STD	= -std=gnu99 $(CFLAGS_SHARED)

LIB_COMMON = $(LIB_PATH)/libcommon.a

LIBCOMMON_OBJS = \
	getopt.o \
	serial.o \
	timestamp.o \
	zarray.o \
	zhash.o

all: $(LIB_COMMON)

%.o: %.c
	@$(CC) $(CFLAGS_STD) -c $<
	@echo "Compiled: $@"

$(LIB_COMMON): $(LIBCOMMON_OBJS) $(LIBDEPS)
	@echo "Made: optitrack/common/lib/libcommon.a"
	@ar rc $@ $^


clean:
	@rm -f *.o *~ *.a
	@rm -f $(LIB_COMMON)
	@echo "optitrack/common/lib/libcommon.a Clean Complete"
