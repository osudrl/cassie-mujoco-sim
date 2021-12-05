# cassie-mujoco-sim
A simulation library for Agility Robotics' Cassie robot using MuJoCo.

Developed and tested on Ubuntu 16.04. Also compiles for Windows using mingw-w64.

To build the shared library:
1.  Install libglfw3-dev
2.  Clone this repository
3.  Download mujoco210-linux-x86_64.tar.gz from mujoco.org
4.  Put mujoco210 in the cassie-mujoco-sim directory
5.  `make`

To cross-compile for Windows, use `make PLATFORM=WIN`.

A directory named release will be created containing the files needed to use the library with a C interface. A Python interface can be generated with `make ctypes`, which requires ctypeslib2 to be installed.

To build and run the examples:
1.  `make test`
2.  Run the examples in the test directory

The examples include cassiesim, which simulates a physical Cassie robot controlled over UDP, and cassiectrl, a null controller operating over UDP. The file cassietest.c is a minimal example of running the simulation, and cassietest.py demonstrates controlling the simulated robot in Python.

Documentation for the simulation functions is included in include/cassiemujoco.h. The file header/udp.h declares convenience functions for creating a UDP socket and obtaining the most recent packet. The remaining header files include functions for packing and unpacking structures used to communicate with Cassie into UDP packets and for simulating Cassie's internal safety mechanisms and state estimation processes.
