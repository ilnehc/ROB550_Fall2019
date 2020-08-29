TARGET = ../bin/test_xbee_receive
LDFLAGS_STD	:= -lm -lpthread
LDFLAGS = $(LDFLAGS_STD) 
CC		:= gcc
LINKER		:= gcc
CFLAGS		:= -std=gnu99 -g -c -Wall
SOURCES		:= $(wildcard *.c)
SOURCES		:= $(SOURCES) $(wildcard ../xbee_serial/*.c)
INCLUDES	:= $(wildcard ../xbee_serial/*.h)
OBJECTS		:= $(SOURCES:$%.c=$%.o)

# linking Objects
$(TARGET): $(OBJECTS)
	@$(LINKER) -o $@ $(OBJECTS) $(LDFLAGS)
	@echo "Made: $@"

# compiling command
$(OBJECTS): %.o : %.c $(INCLUDES)
	@$(CC) $(CFLAGS) $< -o $@
	@echo "Compiled: $@"

all:
	$(TARGET)

clean:
	@rm -f *.o *~ *.a
	@rm -f $(TARGET)
	@echo "$(TARGET) Clean Complete"
