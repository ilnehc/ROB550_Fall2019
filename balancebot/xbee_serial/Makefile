LDFLAGS_STD	:= -lm -lpthread
LDFLAGS = $(LDFLAGS_STD) 
CC		:= gcc
LINKER		:= gcc
CFLAGS		:= -std=gnu99 -g -c -Wall
SOURCES		:= $(wildcard *.c)
INCLUDES	:= $(wildcard *.h)
OBJECTS		:= $(SOURCES:$%.c=$%.o)

# compiling command
$(OBJECTS): %.o : %.c $(INCLUDES)
	@$(CC) $(CFLAGS) $< -o $@
	@echo "Compiled: $@"

all:	$(TARGET)

clean:
	@rm -f *.o *~ *.a
	@echo "../xbee_serial Clean Complete"
