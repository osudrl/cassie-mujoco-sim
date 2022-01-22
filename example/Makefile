# Build for linux by default
PLATFORM := LINUX

# Compilation settings
INC     := -Iinclude -I../include -I$(HOME)/.mujoco/mujoco210/include
CFLAGS  := -std=gnu11 -Wall -Wextra -O3 -march=sandybridge -flto -ggdb3
LDFLAGS := -L. -L..
LIBS    := -lcassiemujoco

# Platform-specific settings
ifeq ($(PLATFORM), WIN)
CC      := x86_64-w64-mingw32-gcc
TESTOUT := cassietest.exe
SIMOUT  := cassiesim.exe
CTRLOUT := cassiectrl.exe
VIDOUT := cassievideo.exe
else
CC      := gcc
LDFLAGS += -Wl,-rpath,'$$ORIGIN'
TESTOUT := cassietest
SIMOUT  := cassiesim
CTRLOUT := cassiectrl
VIDOUT := cassievideo
DOUBLEVISOUT := test_doublevis
HFIELDOUT := test_hfield
TERRAINOUT := test_terrain
FORCEOUT := test_heelforce
endif

# Default target
all: $(TESTOUT) $(SIMOUT) $(CTRLOUT) $(VIDOUT) vel_check $(DOUBLEVISOUT) $(HFIELDOUT) $(TERRAINOUT) $(FORCEOUT)

# Normal targets
clean:
	rm -f $(TESTOUT)
	rm -f $(SIMOUT)
	rm -f $(CTRLOUT)
	rm -f $(VIDOUT)
	rm -f $(DOUBLEVISOUT)
	rm -f $(HFIELDOUT)
	rm -f vel_check

$(TESTOUT): cassietest.c
	$(CC) cassietest.c $(INC) $(CFLAGS) $(LDFLAGS) $(LIBS) -o $(TESTOUT)

$(SIMOUT): cassiesim.c
	$(CC) cassiesim.c $(INC) $(CFLAGS) $(LDFLAGS) $(LIBS) -o $(SIMOUT)

$(CTRLOUT): cassiectrl.c
	$(CC) cassiectrl.c $(INC) $(CFLAGS) $(LDFLAGS) $(LIBS) -o $(CTRLOUT)

$(VIDOUT): cassievideo.c 
	$(CC) cassievideo.c $(INC) $(CFLAGS) $(LDFLAGS) $(LIBS) -o $(VIDOUT)
	
$(DOUBLEVISOUT): test_doublevis.c 
	$(CC) test_doublevis.c $(INC) $(CFLAGS) $(LDFLAGS) $(LIBS) -o $(DOUBLEVISOUT)

$(HFIELDOUT): test_hfield.c 
	$(CC) test_hfield.c $(INC) $(CFLAGS) $(LDFLAGS) $(LIBS) -o $(HFIELDOUT)

$(TERRAINOUT): test_terrain.c 
	$(CC) test_terrain.c $(INC) $(CFLAGS) $(LDFLAGS) $(LIBS) -o $(TERRAINOUT)

$(FORCEOUT): test_heelforce.c 
	$(CC) test_heelforce.c $(INC) $(CFLAGS) $(LDFLAGS) $(LIBS) -o $(FORCEOUT)

vel_check: vel_check.c 
	$(CC) vel_check.c $(INC) $(CFLAGS) $(LDFLAGS) $(LIBS) -o vel_check



# Virtual targets
.PHONY: all clean
