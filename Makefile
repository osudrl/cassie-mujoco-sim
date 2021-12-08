# Build for linux by default
PLATFORM := LINUX

# Compilation settings
INC     := -Iinclude -I$(HOME)/.mujoco/mujoco210/include
CFLAGS  := -std=gnu11 -Wall -Wextra -O3 -march=sandybridge -flto
LDFLAGS := -shared -Lsrc

# Platform-specific settings
ifeq ($(PLATFORM), WIN)
CC     := x86_64-w64-mingw32-gcc
LIBS   := -lws2_32 -Wl,--whole-archive -l:agilitycassie.lib -Wl,--no-whole-archive
LIBOUT := cassiemujoco.dll
else
CC     := gcc
CFLAGS += -fPIC
LIBS   := -lm -ldl -Wl,--whole-archive -lagilitycassie -Wl,--no-whole-archive
LIBOUT := libcassiemujoco.so
endif

# Default build target
all: checkdirs build

# Normal targets
clean:
	rm -f $(LIBOUT)
	rm -rf build/
	rm -rf test/

$(LIBOUT): src/*.c
	echo src/*.c $(INC) $(CFLAGS) -o $(LIBOUT) $(LDFLAGS) $(LIBS)
	$(CC) src/*.c $(INC) $(CFLAGS) -o $(LIBOUT) $(LDFLAGS) $(LIBS)

build: $(LIBOUT)
	mkdir -p build
	cp $(LIBOUT) build/
	cp $(LIBOUT) example/
	cp -r include build/
	cp -r model/* build/

ctypes: build
	clang2py include/*.h --clang-args="-I/usr/include/clang/6.0/include -Iinclude" -l ./$(LIBOUT) -o build/cassiemujoco_ctypes.py
	sed -i '/import ctypes/aimport os\n_dir_path = os.path.dirname(os.path.realpath(__file__))' build/cassiemujoco_ctypes.py
	sed -i "s/CDLL('.\/$(LIBOUT)')/CDLL(_dir_path + '\/$(LIBOUT)')/g" build/cassiemujoco_ctypes.py

test: checkdirs build
	mkdir -p test
	cp -r build/* test/
	cp -r example/* test/
	make -C test PLATFORM="$(PLATFORM)"

# Virtual targets
.PHONY: all checkdirs clean build builddir test ctypes
