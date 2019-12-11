# Build for linux by default
PLATFORM := LINUX

# Compilation settings
INC     := -Iinclude -I../include
CFLAGS  := -std=gnu11 -Wall -Wextra -O3 -march=sandybridge -flto
LDFLAGS := -L. -L..
LIBS    := -lcassiemujoco

# Platform-specific settings
ifeq ($(PLATFORM), WIN)
CC      := x86_64-w64-mingw32-gcc
TESTOUT := cassietest.exe
SIMOUT  := cassiesim.exe
CTRLOUT := cassiectrl.exe
else
CC      := gcc
LDFLAGS += -Wl,-rpath,'$$ORIGIN'
TESTOUT := cassietest
SIMOUT  := cassiesim
CTRLOUT := cassiectrl
endif

# Default target
all: $(TESTOUT) $(SIMOUT) $(CTRLOUT)

# Normal targets
clean:
	rm -f $(TESTOUT)
	rm -f $(SIMOUT)
	rm -f $(CTRLOUT)

$(TESTOUT): cassietest.c
	$(CC) cassietest.c $(INC) $(CFLAGS) $(LDFLAGS) $(LIBS) -o $(TESTOUT)

$(SIMOUT): cassiesim.c
	$(CC) cassiesim.c $(INC) $(CFLAGS) $(LDFLAGS) $(LIBS) -o $(SIMOUT)

$(CTRLOUT): cassiectrl.c
	$(CC) cassiectrl.c $(INC) $(CFLAGS) $(LDFLAGS) $(LIBS) -o $(CTRLOUT)

# Virtual targets
.PHONY: all clean
