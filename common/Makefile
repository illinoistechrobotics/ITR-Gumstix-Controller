# use sdl libraries and lib math
LIBS += -lSDL -pthread `sdl-config --libs`
INCLUDES += -I.
OPTS += -Wall -g
CC ?= gcc
CFLAGS += $(INCLUDES) $(OPTS) `sdl-config --cflags`

BINARY = robot_queue_test
COMMON_OBJ = robot_queue.o \
			 robot_queue_test.o \
			 timer.o

OBJ = $(COMMON_OBJ)

all: $(BINARY)

$(BINARY): $(COMMON_OBJ)
	$(CC) $(CFLAGS) $(LIBS) $(COMMON_OBJ) -o $(BINARY)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	-rm -f $(OBJ) $(BINARY)
