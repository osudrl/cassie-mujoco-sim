CC      := gcc
LD      := $(CC)
INC     := -Iinclude -Imjpro150/include
CFLAGS  := -std=gnu11 -Wall -Wextra -O3 -march=sandybridge -flto -fPIC
LDFLAGS := -shared -Lsrc
LIBS    := -lm -ldl -Wl,--whole-archive -lagilitycassie -Wl,--no-whole-archive
LIBOUT  := libcassiemujoco.so

# Default build target
all: checkdirs build

# Normal targets
clean:
	rm -f $(LIBOUT)
	rm -rf build/
	rm -rf test/

$(LIBOUT):
	gcc src/*.c $(INC) $(CFLAGS) -o $(LIBOUT) $(LDFLAGS) $(LIBS)

build: $(LIBOUT)
	mkdir -p build
	cp $(LIBOUT) build/
	cp -r include build/
	cp -r model/* build/

ctypes: build
	clang2py include/*.h --clang-args="-I/usr/include/clang/6.0/include -Iinclude" -l ./$(LIBOUT) -o build/cassiemujoco_ctypes.py
	sed -i '/import ctypes/aimport os\n_dir_path = os.path.dirname(os.path.realpath(__file__))' build/cassiemujoco_ctypes.py
	sed -i "s/CDLL('.\/$(LIBOUT)')/CDLL(_dir_path + '\/$(LIBOUT)')/g" build/cassiemujoco_ctypes.py

test: checkdirs build ctypes
	mkdir -p test
	cp -r build/* test/
	cp example/* test/
	cp -r mjpro150 test/
	cp mjkey.txt test/
	make -C test

# Virtual targets
.PHONY: all checkdirs clean build builddir test ctypes
