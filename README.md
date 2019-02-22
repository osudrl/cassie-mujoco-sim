# cassie-mujoco-sim

A simulation library for Agility Robotics' Cassie robot using MuJoCo.

Developed and tested on Ubuntu 16.04. Also compiles for Windows using mingw-w64.

To build the shared library:
1.  Install libglfw3-dev
2.  Clone this repository
3.  Download mjpro200_linux.zip from roboti.us
4.  Put mjpro200_linux in a `.mujoco` directory in your home directory
5.  `make`

To cross-compile for Windows, use `make PLATFORM=WIN`. (NOT TESTED)

A directory named release will be created containing the files needed to use the library with a C interface. A Python interface can be generated with `make ctypes`, which requires ctypeslib2 to be installed.

A license file for MuJoCo (mjkey.txt) is required to run the simulation. The library also includes functions that can be used for communicating with Cassie over UDP, and MuJoCo is not required if only these functions are called.

To build and run the examples:
1.  Place mjkey.txt in the `.mujoco`directory
2.  Set the `MUJOCO_KEY_PATH` environment variable to be `~/.mujoco/mjkey.txt`
3.  (Optional) Set the `CASSIE_MODEL_PATH` environment variable to be `<where you cloned this repository>/model/cassie.xml`
4.  `make test`
5.  Run the examples in the test directory

The examples include cassiesim, which simulates a physical Cassie robot controlled over UDP, and cassiectrl, a null controller operating over UDP. The file cassietest.c is a minimal example of running the simulation, and cassietest.py demonstrates controlling the simulated robot in Python.

Documentation for the simulation functions is included in include/cassiemujoco.h. The file header/udp.h declares convenience functions for creating a UDP socket and obtaining the most recent packet. The remaining header files include functions for packing and unpacking structures used to communicate with Cassie into UDP packets and for simulating Cassie's internal safety mechanisms and state estimation processes.

## Changes:
* Updated libraries to use mujoco 2.00
* MuJoCo activation key is now loaded by the `MUJOCO_KEY_PATH` environment variable. Add `export MUJOCO_KEY_PATH = <path to mjkey.txt>` to your `.bashrc`.
* To make things cleaner when loading MuJoCo and GLFW libraries, we assume the `mujoco200_linux` folder is `~/.mujoco` folder. This allows there to be only one copy of mujoco on your machine, rather than having to copy the mujoco folder to every place where `libcassiemujoco.so` is. 
* There are now two options for specifying a Cassie model file. The library will check if there is a `CASSIE_MODEL_PATH` environment variable defined. If such a variable is defined, it loads the model on that path. Otherwise, it will load the modelfile that is inputted into `cassie_sim_init`/`cassie_vis_init` (which is then passed along to `cassie_mujoco_init`). If you are using a environment variable then you can just pass `NULL` or an empty string to these methods. Using an environment variable is makes the usage invariant to the directory a program is run from, but requires changing and sourcing your `.bashrc` everytime you want to change modelfile. Note that a model file path (or `NULL` input) is also required when making a python `CassieSim` or `CassieVis` object, though you can make the argument have a default value so you don't have to input a model everytime.
* If building fails because it can't find `mujoco.h`, specify the full path in the Makefile when including mujoco200_linux/include, rather than using `~`.

## Features to be added:
* Additional key command functionality:
    * backspace key for resetting the model
    * forward key for frame by frame stepping
    * ctrl-q to quit/close the rendering window
* Overlayed graphs of sensor values and ground reaction forces
* Switching between free camera and tracking camera
* Pause and slowmotion functionality (requires changes to `cassiemujoco.py`)