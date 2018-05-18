# Compiler
CC      := gcc
LD      := $(CC)

# Options for all targets
INC     := -Iinclude
CFLAGS  := -std=gnu11 -Wall -Wextra -O3 -march=sandybridge -flto
LDFLAGS := -L. -Wl,-rpath,'$$ORIGIN'
LIBS    := -lcassiemujoco

# Output binaries
TESTOUT := cassietest
SIMOUT  := cassiesim
CTRLOUT := cassiectrl

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
