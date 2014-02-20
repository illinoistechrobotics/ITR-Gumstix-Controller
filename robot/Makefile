# Setup the environment
BUILDROOT_DIR ?= /opt/gumstix-buildroot
PATH := $(BUILDROOT_DIR)/build_arm_nofpu/staging_dir/bin/:$(PATH)
ROBOSTIX_DIR ?= ../robostix/
I2CIO_DIR ?= $(ROBOSTIX_DIR)/gumstix/i2c-io/
COMMON = ../common

# Turn on all warnings
OPTS = -Wall -g -march=armv5te -mtune=xscale
CC = arm-linux-gcc
LIBS = -lpthread -lc

BINARY = robot

LOCAL_OBJ_ROSLUND = robot_events_roslund.o
LOCAL_OBJ_COLONEL = robot_events_colonel.o
LOCAL_OBJ_FENRIR = robot_events_fenrir.o

LOCAL_OBJ  = robot.o \
			 mod_i2c-io.o \
			 adc.o 
COMMON_OBJ = robot_comm.o \
			 robot_log.o \
			 robot_queue.o \
			 timer.o 
I2CIO_OBJ  = AvrInfo.o \
			 BootLoader-api.o \
			 Crc8.o \
			 DumpMem.o \
			 Log.o \
			 i2c-api.o \
			 i2c-io-api.o

# look for .h files in these directories
INCLUDES = -I. \
		   -I$(COMMON) \
		   -I$(ROBOSTIX_DIR)/gumstix/Shared \
		   -I$(ROBOSTIX_DIR)/gumstix/Common \
		   -I$(ROBOSTIX_DIR)/Shared \
		   -I$(ROBOSTIX_DIR)/Common \
		   -I$(ROBOSTIX_DIR)/gumstix/i2c-io \
		   -L$(BUILDROOT_DIR)/build_arm_nofpu/staging_dir/lib/


	   

# Begin derived variablse
CFLAGS += $(INCLUDES) $(OPTS)
LOCAL_SRC = $(LOCAL_OBJ:%.o=%.c)
COMMON_SRC = $(COMMON_OBJ: %.o=$(COMMON)/%.c)

OBJ_ROSLUND = $(LOCAL_OBJ) $(LOCAL_OBJ_ROSLUND) $(COMMON_OBJ) $(I2CIO_OBJ)
OBJ_COLONEL = $(LOCAL_OBJ) $(LOCAL_OBJ_COLONEL) $(COMMON_OBJ) $(I2CIO_OBJ)
OBJ_FENRIR = $(LOCAL_OBJ) $(LOCAL_OBJ_FENRIR) $(COMMON_OBJ) $(I2CIO_OBJ)
SRC = $(LOCAL_SRC) $(COMMON_SRC)

.PHONY: all
all: $(BINARY)-roslund $(BINARY)-colonel $(BINARY)-fenrir

$(BINARY)-roslund: $(OBJ_ROSLUND)
	$(CC) $(CFLAGS) $(INCLUDES) $(LIBS) $(OBJ_ROSLUND) -o $(BINARY)-roslund

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@
%.o: $(COMMON)/%.c
	$(CC) $(CFLAGS) -c $< -o $@
%.o: $(ROBOSTIX_DIR)/gumstix/Shared/%.c
	$(CC) $(CFLAGS) -c $< -o $@
%.o: $(ROBOSTIX_DIR)/gumstix/Common/%.c
	$(CC) $(CFLAGS) -c $< -o $@
%.o: $(ROBOSTIX_DIR)/Shared/%.c
	$(CC) $(CFLAGS) -c $< -o $@
%.o: $(ROBOSTIX_DIR)/Common/%.c
	$(CC) $(CFLAGS) -c $< -o $@
%.o: $(ROBOSTIX_DIR)/gumstix/i2c-io/%.c
	$(CC) $(CFLAGS) -c $< -o $@

.PHONY: clean
clean:
	-rm -f $(OBJ_ROSLUND) $(OBJ_COLONEL) $(OBJ_FENRIR) $(BINARY) $(BINARY)-roslund $(BINARY)-colonel $(BINARY)-fenrir

.PHONY: roslund
roslund: $(BINARY)-roslund

.PHONY: colonel
colonel: $(BINARY)-colonel

$(BINARY)-colonel: $(OBJ_COLONEL)
	$(CC) $(CFLAGS) $(INCLUDES) $(LIBS) $(OBJ_COLONEL) -o $(BINARY)-colonel

.PHONY: fenrir
fenrir: $(BINARY)-fenrir

$(BINARY)-fenrir: $(OBJ_FENRIR)
	$(CC) $(CFLAGS) $(INCLUDES) $(LIBS) $(OBJ_FENRIR) -o $(BINARY)-fenrir
