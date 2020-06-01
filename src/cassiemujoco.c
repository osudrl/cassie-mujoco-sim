/*
 * Copyright (c) 2018 Dynamic Robotics Laboratory
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include "cassiemujoco.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <sys/types.h>
#include <pwd.h>
#include <time.h>
#include <linux/limits.h>
#include "mujoco.h"
#include "glfw3.h"
#include "mjxmacro.h"
#include "uitools.h"
#include "cassie_core_sim.h"
#include "state_output.h"
#include "pd_input.h"

// Platform specific headers
#ifdef _WIN32
#include <windows.h>
#else
#include <dlfcn.h>
#include <libgen.h>
#include <unistd.h>
#endif


/*******************************************************************************
 * Global library state
 ******************************************************************************/

static bool glfw_initialized = false;
static bool mujoco_initialized = false;
static mjModel *initial_model;
static int left_foot_body_id;
static int right_foot_body_id;
// Globals for visualization
static int fontscale = mjFONTSCALE_200;
mjvFigure figsensor;


/*******************************************************************************
 * Dynamic library loading
 ******************************************************************************/

// Loaded MuJoCo functions
#define MUJOCO_FUNCTION_LIST                    \
    X(mj_activate)                              \
    X(mj_deactivate)                            \
    X(mj_loadXML)                               \
    X(mj_copyModel)                             \
    X(mj_deleteModel)                           \
    X(mj_makeData)                              \
    X(mj_copyData)                              \
    X(mj_deleteData)                            \
    X(mj_resetData)                             \
    X(mj_forward)                               \
    X(mj_setConst)                              \
    X(mj_fwdPosition)                           \
    X(mj_comVel)                                \
    X(mj_step1)                                 \
    X(mj_step2)                                 \
    X(mj_step)                                  \
    X(mj_contactForce)                          \
    X(mj_name2id)                               \
    X(mju_copy)                                 \
    X(mju_zero)                                 \
    X(mju_rotVecMatT)                           \
    X(mju_sub3)                                 \
    X(mju_mulMatTVec)                           \
    X(mju_printMat)                             \
    X(mjv_makeScene)                            \
    X(mjv_defaultScene)                         \
    X(mjv_freeScene)                            \
    X(mjv_updateScene)                          \
    X(mjv_defaultCamera)                        \
    X(mjv_defaultOption)                        \
    X(mjv_movePerturb)                          \
    X(mjv_moveCamera)                           \
    X(mjv_initPerturb)                          \
    X(mjv_select)                               \
    X(mjv_applyPerturbForce)                    \
    X(mjv_applyPerturbPose)                     \
    X(mjr_defaultContext)                       \
    X(mjv_defaultFigure)                        \
    X(mjr_makeContext)                          \
    X(mjr_freeContext)                          \
    X(mjr_render)                               \
    X(mjr_overlay)                              \
    X(mjr_figure)

// Loaded GLFW functions
#define GLFW_FUNCTION_LIST                      \
    X(glfwInit)                                 \
    X(glfwTerminate)                            \
    X(glfwCreateWindow)                         \
    X(glfwDestroyWindow)                        \
    X(glfwMakeContextCurrent)                   \
    X(glfwGetWindowUserPointer)                 \
    X(glfwSetWindowUserPointer)                 \
    X(glfwSetWindowCloseCallback)               \
    X(glfwSetCursorPosCallback)                 \
    X(glfwSetMouseButtonCallback)               \
    X(glfwSetScrollCallback)                    \
    X(glfwSetKeyCallback)                       \
    X(glfwGetFramebufferSize)                   \
    X(glfwSwapBuffers)                          \
    X(glfwSwapInterval)                         \
    X(glfwPollEvents)                           \
    X(glfwGetVideoMode)                         \
    X(glfwGetPrimaryMonitor)                    \
    X(glfwGetWindowSize)                        \
    X(glfwGetKey)                               \
    X(glfwGetMouseButton)                       \
    X(glfwGetCursorPos)                         \
    X(glfwRestoreWindow)                        \
    X(glfwMaximizeWindow)                       \
    X(glfwSetWindowShouldClose)                 \
    X(glfwWindowShouldClose)

// Dynamic object handles
static void *mj_handle;
static void *glfw_handle;
static void *gl_handle;
static void *glew_handle;

// Function pointers
#define X(fun)                                  \
    typedef __typeof__(fun) fun ## _fp_type;    \
    static fun ## _fp_type *fun ## _fp;
MUJOCO_FUNCTION_LIST
GLFW_FUNCTION_LIST
#undef X

// Cross-platform dynamic loading
#ifdef _WIN32

#define LOADLIB(path) LoadLibrary(path)
#define UNLOADLIB(handle) FreeLibrary(handle)
#define LOADFUN(handle, sym) sym ## _fp = (void*) GetProcAddress(handle, #sym)
#define MJLIBNAME "mujoco150.dll"
#define GLFWLIBNAME "glfw3.dll"

#else

#define LOADLIB(path) dlopen(path, RTLD_LAZY | RTLD_GLOBAL)
#define UNLOADLIB(handle) dlclose(handle)
#define LOADFUN(handle, sym) sym ## _fp = dlsym(handle, #sym)
#define MJLIBNAME "libmujoco200.so"
#define MJLIBNAMENOGL "libmujoco200nogl.so"
#define GLFWLIBNAME "libglfw.so.3"

#endif

/*******************************************************************************
 * Sensor filtering
 ******************************************************************************/

#define DRIVE_FILTER_NB 9
#define JOINT_FILTER_NB 4
#define JOINT_FILTER_NA 3

static int drive_filter_b[DRIVE_FILTER_NB] = {
    2727, 534, -2658, -795, 72, 110, 19, -6, -3
};

static double joint_filter_b[JOINT_FILTER_NB] = {
    12.348, 12.348, -12.348, -12.348
};

static double joint_filter_a[JOINT_FILTER_NA] = {
    1.0, -1.7658, 0.79045
};

typedef struct drive_filter {
    int x[DRIVE_FILTER_NB];
} drive_filter_t;

typedef struct joint_filter {
    double x[JOINT_FILTER_NB];
    double y[JOINT_FILTER_NA];
} joint_filter_t;


/*******************************************************************************
 * Drive and joint order X macro lists
 ******************************************************************************/

#define DRIVE_LIST                              \
    X(leftLeg.hipRollDrive)                     \
    X(leftLeg.hipYawDrive)                      \
    X(leftLeg.hipPitchDrive)                    \
    X(leftLeg.kneeDrive)                        \
    X(leftLeg.footDrive)                        \
    X(rightLeg.hipRollDrive)                    \
    X(rightLeg.hipYawDrive)                     \
    X(rightLeg.hipPitchDrive)                   \
    X(rightLeg.kneeDrive)                       \
    X(rightLeg.footDrive)

#define JOINT_LIST                              \
    X(leftLeg.shinJoint)                        \
    X(leftLeg.tarsusJoint)                      \
    X(leftLeg.footJoint)                        \
    X(rightLeg.shinJoint)                       \
    X(rightLeg.tarsusJoint)                     \
    X(rightLeg.footJoint)


/*******************************************************************************
 * Opaque structure definitions
 ******************************************************************************/

#define NUM_DRIVES 10
#define NUM_JOINTS 6
#define TORQUE_DELAY_CYCLES 6

struct cassie_sim {
    mjModel *m;
    mjData *d;
    cassie_core_sim_t *core;
    state_output_t *estimator;
    pd_input_t *pd;
    cassie_out_t cassie_out;
    drive_filter_t drive_filter[NUM_DRIVES];
    joint_filter_t joint_filter[NUM_JOINTS];
    double torque_delay[NUM_DRIVES][TORQUE_DELAY_CYCLES];
};

struct cassie_vis {
    //visual interaction controls
    double lastx;
    double lasty;
    bool button_left;
    bool button_middle;
    bool button_right;

    int lastbutton;
    double lastclicktm;

    int refreshrate;

    int showhelp;
    bool showoption;
    bool showGRF;
    int GRFcount;
    bool showfullscreen;
    bool showsensor;
    bool slowmotion;

    bool showinfo;
    bool paused;

    int framenum;
    int lastframenum;

    int perturb_body;   // Body to apply perturb force to in vis_draw
    double perturb_force[6];    // Perturb force to apply
    
    // GLFW  handle
    GLFWwindow *window;

    // MuJoCo stuff

    mjvCamera cam;
    mjvOption opt;
    mjvScene scn;
    mjrContext con;
    mjvPerturb pert;
    mjvFigure figsensor;
    mjvFigure figGRF;
    mjModel* m;
    mjData* d;
};

struct cassie_state {
    mjData *d;
    cassie_core_sim_t *core;
    state_output_t *estimator;
    pd_input_t *pd;
    cassie_out_t cassie_out;
    drive_filter_t drive_filter[NUM_DRIVES];
    joint_filter_t joint_filter[NUM_JOINTS];
    double torque_delay[NUM_DRIVES][TORQUE_DELAY_CYCLES];
};

// Redefine mjVISSTRING HAAAACKKKK to fix stupid bug
const char* VISSTRING[mjNVISFLAG][3] = { {"Convex Hull"    ,"0",  "H"},
                                        {"Texture"         ,"1",  "X"},
                                        {"Joint"           ,"0",  "J"},
                                        {"Actuator"        ,"0",  "U"},
                                        {"Camera"          ,"0",  "Q"},
                                        {"Light"           ,"0",  "Z"},
                                        {"Tendon"          ,"1",  "V"},
                                        {"Range Finder"    ,"1",  "Y"},
                                        {"Constraint"      ,"0",  "N"},
                                        {"Inertia"         ,"0",  "I"},
                                        {"SCL Inertia"     ,"0",  "S"},
                                        {"Perturb Force"   ,"0",  "B"},
                                        {"Perturb Object"  ,"1",  "O"},
                                        {"Contact Point"   ,"0",  "C"},
                                        {"Contact Force"   ,"0",  "F"},
                                        {"Contact Split"   ,"0",  "P"},
                                        {"Transparent"     ,"0",  "T"},
                                        {"Auto Connect"    ,"0",  "A"},
                                        {"Center of Mass"  ,"0",  "M"},
                                        {"Select Point"    ,"0",  "E"},
                                        {"Static Body"     ,"1",  "D"},
                                        {"Skin"            ,"1",  ";"}};
const char* RNDSTRING[mjNRNDFLAG][3] = {{"Shadow"      ,"1",  "S"},
                                        {"Wireframe"   ,"0",  "W"},
                                        {"Reflection"  ,"1",  "R"},
                                        {"Additive"    ,"0",  "L"},
                                        {"Skybox"      ,"1",  "K"},
                                        {"Fog"         ,"0",  "G"},
                                        {"Haze"        ,"1",  "/"},
                                        {"Segment"     ,"0",  ","},
                                        {"Id Color"    ,"0",  "."}};

// help strings
const char help_content[] = 
        "Alt mouse button\n"
        "UI right hold\n"
        "UI title double-click\n"
        "Space\n"
        "Esc\n"
        "Right arrow\n"
        "Left arrow\n"
        "Down arrow\n"
        "Up arrow\n"
        "Page Up\n"
        "Double-click\n"
        "Right double-click\n"
        "Ctrl Right double-click\n"
        "Scroll, middle drag\n"
        "Left drag\n"
        "[Shift] right drag\n"
        "Ctrl [Shift] drag\n"
        "Ctrl [Shift] right drag";

const char help_title[] = 
        "Swap left-right\n"
        "Show UI shortcuts\n"
        "Expand/collapse all  \n"
        "Pause\n"
        "Free camera\n"
        "Step forward\n"
        "Step back\n"
        "Step forward 100\n"
        "Step back 100\n"
        "Select parent\n"
        "Select\n"
        "Center\n"
        "Track camera\n"
        "Zoom\n"
        "View rotate\n"
        "View translate\n"
        "Object rotate\n"
        "Object translate";

#define CASSIE_ALLOC_POINTER(c)                 \
    do {                                        \
        c->d = mj_makeData_fp(initial_model);   \
        c->core = cassie_core_sim_alloc();      \
        c->estimator = state_output_alloc();    \
        c->pd = pd_input_alloc();               \
    } while (0)

#define CASSIE_SIM_ALLOC_POINTER(c)                 \
    do {                                        \
        c->d = mj_makeData_fp(c->m);   \
        c->core = cassie_core_sim_alloc();      \
        c->estimator = state_output_alloc();    \
        c->pd = pd_input_alloc();               \
    } while (0)

#define CASSIE_FREE_POINTER(c)                  \
    do {                                        \
        mj_deleteData_fp(c->d);                 \
        cassie_core_sim_free(c->core);          \
        state_output_free(c->estimator);        \
        pd_input_free(c->pd);                   \
    } while (0)

#define CASSIE_COPY_POD(dst, src)                       \
    do {                                                \
        dst->cassie_out = src->cassie_out;              \
        memcpy(dst->drive_filter, src->drive_filter,    \
               sizeof dst->drive_filter);               \
        memcpy(dst->joint_filter, src->joint_filter,    \
               sizeof dst->joint_filter);               \
        memcpy(dst->torque_delay, src->torque_delay,    \
               sizeof dst->torque_delay);               \
    } while (0)

#define CASSIE_COPY_POINTER(dst, src)                       \
    do {                                                    \
        mj_copyData_fp(dst->d, initial_model, src->d);      \
        cassie_core_sim_copy(dst->core, src->core);         \
        state_output_copy(dst->estimator, src->estimator);  \
        pd_input_copy(dst->pd, src->pd);                    \
    } while (0)

#define CASSIE_SIM_COPY_POINTER(dst, src)                       \
    do {                                                    \
        mj_copyData_fp(dst->d, src->m, src->d);      \
        cassie_core_sim_copy(dst->core, src->core);         \
        state_output_copy(dst->estimator, src->estimator);  \
        pd_input_copy(dst->pd, src->pd);                    \
    } while (0)

/*******************************************************************************
 * Private functions
 ******************************************************************************/

static bool load_glfw_library(const char *basedir)
{
    // Buffer for paths
    char buf[4096 + 1024];

#ifndef _WIN32
    // Open dependencies
    gl_handle = LOADLIB("libGL.so.1");
    snprintf(buf, sizeof buf, "%.4096s/.mujoco/mujoco200_linux/bin/libglew.so", basedir);
    glew_handle = LOADLIB(buf);
    if (!gl_handle || !glew_handle) {
        printf("gl_handle or glew_handle not loaded\n");
        return false;
    }
#endif

    // Open library
    snprintf(buf, sizeof buf, "%.4096s/.mujoco/mujoco200_linux/bin/" GLFWLIBNAME, basedir);
    glfw_handle = LOADLIB(buf);
    if (!glfw_handle) {
        fprintf(stderr, "Failed to load %s\n", buf);
        return false;
    }

    // Get function pointers
#define X(fun) LOADFUN(glfw_handle, fun);
GLFW_FUNCTION_LIST
#undef X

    return true;
}


static bool load_mujoco_library()
{
    // Buffer for paths
    char buf[4096 + 1024];
    // Get home directory
    const char* homedir;
    if ((homedir = getenv("HOME")) == NULL) {
        homedir = getpwuid(getuid())->pw_dir;
    }

    // Try loading GLFW
    bool __attribute__((unused)) gl = load_glfw_library(homedir);
    //struct passwd *pw = getpwuid(getuid());
    
    // Choose library version
    snprintf(buf, sizeof buf, "%.4096s/.mujoco/mujoco200_linux/bin/" MJLIBNAME, homedir);
    // snprintf(buf, sizeof buf, "%.4096s/mujoco200_linux/bin/" MJLIBNAME, "~/.mujoco");
#ifndef _WIN32
    if (!gl) {
        snprintf(buf, sizeof buf, "%.4096s/.mujoco/mujoco200_linux/bin/" MJLIBNAMENOGL, homedir);
        // snprintf(buf, sizeof buf, "%.4096s/.mujoco/mujoco200_linux/bin/" MJLIBNAMENOGL, "~");
    }
#endif

    // Open library
    mj_handle = LOADLIB(buf);
    if (!mj_handle) {
        fprintf(stderr, "Failed to load %s\n%s\n", buf, dlerror());
        return false;
    }

    // Get function pointers
#define X(fun) LOADFUN(mj_handle, fun);
MUJOCO_FUNCTION_LIST
#undef X
    return true;
}


static void drive_encoder(const mjModel *m,
                          elmo_out_t *drive,
                          const mjtNum *sensordata,
                          drive_filter_t *filter,
                          int isensor)
{
    // Position
    // Get digital encoder value
    int bits = m->sensor_user[m->nuser_sensor * isensor];
    int encoder_value = sensordata[isensor] / (2 * M_PI) * (1 << bits);
    double ratio = m->actuator_gear[6 * m->sensor_objid[isensor]];
    double scale = (2 * M_PI) / (1 << bits) / ratio;
    drive->position = encoder_value * scale;

    // Velocity
    // Initialize unfiltered signal array to prevent bad transients
    bool allzero = true;
    for (int i = 0; i < DRIVE_FILTER_NB; ++i)
        allzero &= filter->x[i] == 0;
    if (allzero) {
        // If all filter values are zero, initialize the signal array
        // with the current encoder value
        for (int i = 0; i < DRIVE_FILTER_NB; ++i)
            filter->x[i] = encoder_value;
    }

    // Shift and update unfiltered signal array
    for (int i = DRIVE_FILTER_NB - 1; i > 0; --i)
        filter->x[i] = filter->x[i - 1];
    filter->x[0] = encoder_value;
    // Compute filter value
    int y = 0;
    for (int i = 0; i < DRIVE_FILTER_NB; ++i)
        y += filter->x[i] * drive_filter_b[i];
    drive->velocity = y * scale / M_PI;
}


static void joint_encoder(const mjModel *m,
                          cassie_joint_out_t *joint,
                          const mjtNum *sensordata,
                          joint_filter_t *filter,
                          int isensor)
{
    // Position
    // Get digital encoder value
    int bits = m->sensor_user[m->nuser_sensor * isensor];
    int encoder_value = sensordata[isensor] / (2 * M_PI) * (1 << bits);
    double scale = (2 * M_PI) / (1 << bits);
    joint->position = encoder_value * scale;

    // Velocity
    // Initialize unfiltered signal array to prevent bad transients
    bool allzero = true;
    for (int i = 0; i < JOINT_FILTER_NB; ++i)
        allzero &= filter->x[i] == 0;
    if (allzero) {
        // If all filter values are zero, initialize the signal array
        // with the current encoder value
        for (int i = 0; i < JOINT_FILTER_NB; ++i)
            filter->x[i] = joint->position;
    }

    // Shift and update signal arrays
    for (int i = JOINT_FILTER_NB - 1; i > 0; --i)
        filter->x[i] = filter->x[i - 1];
    filter->x[0] = joint->position;
    for (int i = JOINT_FILTER_NA - 1; i > 0; --i)
        filter->y[i] = filter->y[i - 1];

    // Compute filter value
    filter->y[0] = 0;
    for (int i = 0; i < JOINT_FILTER_NB; ++i)
        filter->y[0] += filter->x[i] * joint_filter_b[i];
    for (int i = 1; i < JOINT_FILTER_NA; ++i)
        filter->y[0] -= filter->y[i] * joint_filter_a[i];
    joint->velocity = filter->y[0];
}


static double motor(const mjModel* m, mjData *d, int i, double u,
                    double *torque_delay, bool sto)
{
    double ratio = m->actuator_gear[6 * i];
    double tmax = m->actuator_ctrlrange[2 * i + 1];
    double w = d->actuator_velocity[i];
    double wmax = m->actuator_user[m->nuser_actuator * i] * 2 * M_PI / 60;

    // Calculate torque limit based on motor speed
    double tlim = 2 * tmax * (1 - fabs(w) / wmax);
    tlim = fmax(fmin(tlim, tmax), 0);

    // Apply STO
    if (sto)
        u = 0;

    // Compute motor-side torque
    double tau = copysign(fmin(fabs(u / ratio), tlim), u);

    // Torque delay line
    d->ctrl[i] = torque_delay[TORQUE_DELAY_CYCLES - 1];
    for (int i = TORQUE_DELAY_CYCLES - 1; i > 0; --i)
        torque_delay[i] = torque_delay[i - 1];
    torque_delay[0] = tau;
    // Return the current value of the output-side torque
    return d->ctrl[i] * ratio;
}


static void window_close_callback(GLFWwindow *window)
{
    cassie_vis_close(glfwGetWindowUserPointer_fp(window));
}


static void elmo_out_init(elmo_out_t *o, double torqueLimit, double gearRatio)
{
    o->statusWord = 0x0637;
    o->dcLinkVoltage = 48;
    o->driveTemperature = 30;
    o->torqueLimit = torqueLimit;
    o->gearRatio = gearRatio;
}


static void cassie_leg_out_init(cassie_leg_out_t *o)
{
    o->medullaCounter = 1;
    o->medullaCpuLoad = 94;
    elmo_out_init(&o->hipRollDrive,  140.63, 25);
    elmo_out_init(&o->hipYawDrive,   140.63, 25);
    elmo_out_init(&o->hipPitchDrive, 216.16, 16);
    elmo_out_init(&o->kneeDrive,     216.16, 16);
    elmo_out_init(&o->footDrive,      45.14, 50);
}


static void cassie_out_init(cassie_out_t *o)
{
    // The struct is zero-initialized when created

    // Calibrated
    o->isCalibrated = true;

    // Pelvis
    o->pelvis.medullaCounter = 1;
    o->pelvis.medullaCpuLoad = 159;
    o->pelvis.vtmTemperature = 40;

    // Target PC
    o->pelvis.targetPc.etherCatStatus[1] = 8;
    o->pelvis.targetPc.etherCatStatus[4] = 1;
    o->pelvis.targetPc.taskExecutionTime = 2e-4;
    o->pelvis.targetPc.cpuTemperature = 60;

    // Battery
    o->pelvis.battery.dataGood = true;
    o->pelvis.battery.stateOfCharge = 1;
    for (int i = 0; i < 4; ++i)
        o->pelvis.battery.temperature[i] = 30;
    for (int i = 0; i < 12; ++i)
        o->pelvis.battery.voltage[i] = 4.2;

    // Radio
    o->pelvis.radio.radioReceiverSignalGood = true;
    o->pelvis.radio.receiverMedullaSignalGood = true;
    o->pelvis.radio.channel[8] = 1;

    // VectorNav
    o->pelvis.vectorNav.dataGood = true;
    o->pelvis.vectorNav.pressure = 101.325;
    o->pelvis.vectorNav.temperature = 25;

    // Legs
    cassie_leg_out_init(&o->leftLeg);
    cassie_leg_out_init(&o->rightLeg);
}


static void cassie_sensor_data(cassie_sim_t *c)
{
    // Ordered list of drive_out_t addresses
    elmo_out_t *drives[NUM_DRIVES] = {
#define X(drive) &c->cassie_out.drive,
        DRIVE_LIST
#undef X
    };

    // Ordered list of cassie_joint_out_t addresses
    cassie_joint_out_t *joints[NUM_JOINTS] = {
#define X(joint) &c->cassie_out.joint,
        JOINT_LIST
#undef X
    };

    // Sensor ID for each encoder
    static const int drive_sensor_ids[] = {0, 1, 2, 3, 4, 8, 9, 10, 11, 12};
    static const int joint_sensor_ids[] = {5, 6, 7, 13, 14, 15};

    // Encoders
    for (int i = 0; i < NUM_DRIVES; ++i)
        drive_encoder(c->m, drives[i], c->d->sensordata,
                      &c->drive_filter[i], drive_sensor_ids[i]);
    for (int i = 0; i < NUM_JOINTS; ++i)
        joint_encoder(c->m, joints[i], c->d->sensordata,
                      &c->joint_filter[i], joint_sensor_ids[i]);

    // IMU
    mju_copy_fp(c->cassie_out.pelvis.vectorNav.orientation,
                &c->d->sensordata[16], 4);
    mju_copy_fp(c->cassie_out.pelvis.vectorNav.angularVelocity,
                &c->d->sensordata[20], 3);
    mju_copy_fp(c->cassie_out.pelvis.vectorNav.linearAcceleration,
                &c->d->sensordata[23], 3);
    mju_copy_fp(c->cassie_out.pelvis.vectorNav.magneticField,
                &c->d->sensordata[26], 3);
}


static void cassie_motor_data(cassie_sim_t *c, const cassie_in_t *cassie_in)
{
    // STO
    bool sto = c->cassie_out.pelvis.radio.channel[8] < 1;

    // Ordered list of drive_out_t addresses
    elmo_out_t *drives[NUM_DRIVES] = {
#define X(drive) &c->cassie_out.drive,
        DRIVE_LIST
#undef X
    };

    // Ordered list of torque commands
    double torque_commands[NUM_DRIVES] = {
#define X(drive) cassie_in->drive.torque,
        DRIVE_LIST
#undef X
    };
    // Copy motor data from cassie out and set torque measurement
    for (int i = 0; i < NUM_DRIVES; ++i)
        drives[i]->torque = motor(c->m, c->d, i, torque_commands[i],
                                  c->torque_delay[i], sto);
}


/*******************************************************************************
 * Public functions
 ******************************************************************************/

#define ID_NAME_LOOKUP(model, idvar, objtype, name)                            \
    do {                                                                \
        idvar = mj_name2id_fp(model, objtype, #name);                   \
        if (-1 == idvar) {                                              \
            fprintf(stderr, "Could not find body named " #name "\n");   \
            return false;                                               \
        }                                                               \
    } while (0)


bool cassie_mujoco_init(const char *file_input)
{
    
    // Check if mujoco has already been initialized
    if (!mujoco_initialized) {
        // If no base directory is provided, use the direectory
        // containing the executable as the base directory
#ifdef _WIN32
        char buf2[1024];
        HMODULE hModule = GetModuleHandle(NULL);
        GetModuleFileName(hModule, buf2, sizeof buf2);
        char bindrive[16];
        _splitpath_s(buf2, bindrive, sizeof bindrive,
                     buf, sizeof buf, NULL, 0, NULL, 0);
        snprintf(buf2, sizeof buf2, "%s%s", bindrive, buf);
        if (!basedir)
            basedir = buf2;
#else
        char binpath[4096];
        if (-1 == readlink("/proc/self/exe", binpath, sizeof binpath))
            fprintf(stderr, "Failed to get binary directory\n");
        // if (!basedir)
        //     basedir = dirname(binpath);
#endif

        // Load MuJoCo
        if (!load_mujoco_library()) {
            return false;
        }
        // Activate MuJoCo
        const char* key_buf = getenv("MUJOCO_KEY_PATH");
        mj_activate_fp(key_buf);
        // Load the model;
        const char* modelfile;
        if ((modelfile = getenv("CASSIE_MODEL_PATH")) == NULL) {
            //printf("env variable doesn't exist\n");
            modelfile = file_input;
        }
        char error[1000] = "Could not load XML model";
        initial_model = mj_loadXML_fp(modelfile, 0, error, 1000); 
        if (!initial_model) {
            fprintf(stderr, "Load model error: %s\n", error);
            return false;
        }
        int sens_objid[20] = {0, 1, 2, 3, 4, 9, 10, 14, 5, 6, 7, 8, 9, 20, 21, 25, 0, 0, 0, 0};
        for (int i = 0; i < 20; i++) {
            initial_model->sensor_objid[i] = sens_objid[i];
        }
        // Look up relevant IDs based on names
        ID_NAME_LOOKUP(initial_model, left_foot_body_id, mjOBJ_BODY, left-foot);
        ID_NAME_LOOKUP(initial_model, right_foot_body_id, mjOBJ_BODY, right-foot);

        mujoco_initialized = true;
    }
    // Initialize GLFW if it was loaded
    if (glfw_handle && !glfw_initialized) {
        if (!glfwInit_fp()) {
            //fprintf(stderr, "Could not initialize GLFW\n");
            return false;
        }
        glfw_initialized = true;
    }
    return mujoco_initialized;
}


void cassie_cleanup()
{
    if (mj_handle) {
        if (mujoco_initialized) {
            if (initial_model) {
                mj_deleteModel_fp(initial_model);
                initial_model = NULL;
            }
            mj_deactivate_fp();
            mujoco_initialized = false;
        }

        UNLOADLIB(mj_handle);
        mj_handle = NULL;
    }

    if (glfw_handle) {
        if (glfw_initialized) {
            glfwTerminate_fp();
            glfw_initialized = false;
        }

        UNLOADLIB(glfw_handle);
        glfw_handle = NULL;
    }

    if (glew_handle) {
        UNLOADLIB(glew_handle);
        glew_handle = NULL;
    }

    if (gl_handle) {
        UNLOADLIB(gl_handle);
        gl_handle = NULL;
    }
}

bool cassie_reload_xml(const char* modelfile) {

    char error[1000] = "Could not load XML model";
    initial_model = mj_loadXML_fp(modelfile, 0, error, 1000); 
    if (!initial_model) {
        fprintf(stderr, "Load model error: %s\n", error);
        return false;
    }
    int sens_objid[20] = {0, 1, 2, 3, 4, 9, 10, 14, 5, 6, 7, 8, 9, 20, 21, 25, 0, 0, 0, 0};
    for (int i = 0; i < 20; i++) {
        initial_model->sensor_objid[i] = sens_objid[i];
    }
    // Look up relevant IDs based on names
    ID_NAME_LOOKUP(initial_model, left_foot_body_id, mjOBJ_BODY, left-foot);
    ID_NAME_LOOKUP(initial_model, right_foot_body_id, mjOBJ_BODY, right-foot);
    return true;
}

cassie_sim_t *cassie_sim_init(const char* modelfile, bool reinit)
{
    // Make sure MuJoCo is initialized and the model is loaded
    if (!mujoco_initialized) {
        if (!cassie_mujoco_init(modelfile)) {
            return NULL;
        }   
    }

    // Allocate memory, zeroed for cassie_out_t and filter initialization
    cassie_sim_t *c = calloc(1, sizeof (cassie_sim_t));
    // Initialize cassie outputs
    cassie_out_init(&c->cassie_out);

    // Filters initialized to zero

    if (reinit) {
        char error[1000] = "Could not load XML model";
        mjModel *new_model = mj_loadXML_fp(modelfile, 0, error, 1000); 
        if (!new_model) {
            fprintf(stderr, "Load model error: %s\n", error);
            return NULL;
        }
        int sens_objid[20] = {0, 1, 2, 3, 4, 9, 10, 14, 5, 6, 7, 8, 9, 20, 21, 25, 0, 0, 0, 0};
        for (int i = 0; i < 20; i++) {
            new_model->sensor_objid[i] = sens_objid[i];
        }
        // Look up relevant IDs based on names
        ID_NAME_LOOKUP(new_model, left_foot_body_id, mjOBJ_BODY, left-foot);
        ID_NAME_LOOKUP(new_model, right_foot_body_id, mjOBJ_BODY, right-foot);
        c->m =  mj_copyModel_fp(NULL, new_model);
    } else {
        // Initialize mjModel
        c->m = mj_copyModel_fp(NULL, initial_model);
    }

    // Allocate pointer types
    CASSIE_SIM_ALLOC_POINTER(c);

    // Set initial joint configuration
    double qpos_init[] =
        { 0.0045, 0, 0.4973, 0.9785, -0.0164, 0.01787, -0.2049,
         -1.1997, 0, 1.4267, 0, -1.5244, 1.5244, -1.5968,
         -0.0045, 0, 0.4973, 0.9786, 0.00386, -0.01524, -0.2051,
         -1.1997, 0, 1.4267, 0, -1.5244, 1.5244, -1.5968};
    mju_copy_fp(&c->d->qpos[7], qpos_init, 28);
    mj_forward_fp(c->m, c->d);

    // Intialize systems
    cassie_core_sim_setup(c->core);
    state_output_setup(c->estimator);
    pd_input_setup(c->pd);
    return c;
}


cassie_sim_t *cassie_sim_duplicate(const cassie_sim_t *src)
{
    // Allocate storage
    cassie_sim_t *c = malloc(sizeof (cassie_sim_t));
    CASSIE_SIM_ALLOC_POINTER(c);

    // Copy data
    cassie_sim_copy(c, src);

    return c;
}


void cassie_sim_copy(cassie_sim_t *dst, const cassie_sim_t *src)
{
    // Copy POD types
    CASSIE_COPY_POD(dst, src);

    // Copy pointer types
    mj_copyModel_fp(dst->m, src->m);
    CASSIE_SIM_COPY_POINTER(dst, src);
}


void cassie_sim_free(cassie_sim_t *c)
{
    if (!c)
        return;

    // Free pointer elements
    CASSIE_FREE_POINTER(c);
    mj_deleteModel_fp(c->m);

    // Free cassie_sim_t
    free(c);
}


void cassie_sim_step_ethercat(cassie_sim_t *c,
                              cassie_out_t *y,
                              const cassie_in_t *u)
{
    // Configured to emulate delay on the physical robot
    // Corresponds to running a controller directly in Simulink
    // Apply control signal to MuJoCo control inputs
    cassie_motor_data(c, u);

    // Get measurement data using current MuJoCo state, before new
    // control input is actually applied
    cassie_sensor_data(c);
    *y = c->cassie_out;

    // Step the MuJoCo simulation forward
    const int mjsteps = round(5e-4 / c->m->opt.timestep);
    for (int i = 0; i < mjsteps; ++i) {
        mj_step1_fp(c->m, c->d);
        mj_step2_fp(c->m, c->d);
    }
}


void cassie_sim_step(cassie_sim_t *c, cassie_out_t *y, const cassie_user_in_t *u)
{
    // Run cassie core system to get internal cassie inputs
    cassie_in_t cassie_in;
    cassie_core_sim_step(c->core, u, &c->cassie_out, &cassie_in);

    // Run ethercat-level simulator
    cassie_sim_step_ethercat(c, y, &cassie_in);
}


int test = 0;
void cassie_sim_step_pd(cassie_sim_t *c, state_out_t *y, const pd_in_t *u)
{
    // Run PD controller system
    cassie_user_in_t cassie_user_in;
    pd_input_step(c->pd, u, &c->cassie_out, &cassie_user_in);
    // Run core-level simulator
    cassie_out_t cassie_out;
    cassie_sim_step(c, &cassie_out, &cassie_user_in);
    // Run state estimator system
    state_output_step(c->estimator, &cassie_out, y);
}


double *cassie_sim_time(cassie_sim_t *c)
{
    return &c->d->time;
}


double *cassie_sim_qpos(cassie_sim_t *c)
{
    return c->d->qpos;
}


double *cassie_sim_qvel(cassie_sim_t *c)
{
    return c->d->qvel;
}

double *cassie_sim_qacc(cassie_sim_t *c)
{
    return c->d->qacc;
}

int cassie_sim_forward(cassie_sim_t *c)
{
   mj_forward_fp(c->m, c->d);
   return 0;
}

double *cassie_sim_accel(cassie_sim_t *c)
{
    return c->d->qacc;
}


double *cassie_sim_qfrc(cassie_sim_t *c)
{
    return c->d->qfrc_applied;
}


double *cassie_sim_ctrl(cassie_sim_t *c)
{
    return c->d->ctrl;
}

double* cassie_sim_xquat(cassie_sim_t *c, const char* name)
{
    int body_id = mj_name2id_fp(c->m, mjOBJ_BODY, name);
    return &(c->d->xquat[4*body_id]);
}

double *cassie_sim_dof_damping(cassie_sim_t *c)
{
    return c->m->dof_damping;
}

double *cassie_sim_body_mass(cassie_sim_t *c)
{
    return c->m->body_mass;
}

double *cassie_sim_body_ipos(cassie_sim_t *c)
{
    return c->m->body_ipos;
}

double *cassie_sim_geom_friction(cassie_sim_t *c)
{
    return c->m->geom_friction;
}

void cassie_sim_setctrl(cassie_sim_t *c, double *ctrl)
{
    for (int i = 0; i < c->m->nu; i++) {
        c->d->ctrl[i] = ctrl[i];
    }
}

void cassie_sim_set_dof_damping(cassie_sim_t *c, double *damp)
{
    for (int i = 0; i < c->m->nv; i++) {
        c->m->dof_damping[i] = damp[i];
    }
}

void cassie_sim_set_body_mass(cassie_sim_t *c, double *mass)
{
    for (int i = 0; i < c->m->nbody; i++) {
        c->m->body_mass[i] = mass[i];
    }
}

void cassie_sim_set_body_ipos(cassie_sim_t *c, double *ipos)
{
    for (int i = 0; i < c->m->nbody; i++) {
        c->m->body_ipos[i] = ipos[i];
    }
}

void cassie_sim_set_geom_friction(cassie_sim_t *c, double *fric)
{
    for (int i = 0; i < c->m->ngeom*3; i++){
        c->m->geom_friction[i] = fric[i];
    }
}

void cassie_sim_set_geom_name_friction(cassie_sim_t *c, const char* name, double *fric)
{
    int geom_id = mj_name2id_fp(initial_model, mjOBJ_GEOM, name);
    mju_copy_fp(&c->m->geom_friction[geom_id], fric, 3);
}

void cassie_sim_set_const(cassie_sim_t *c)
{
    mj_setConst_fp(c->m, c->d);
    double qpos_init[35] =
        {0, 0, 1.01, 1, 0, 0, 0,
        0.0045, 0, 0.4973, 0.9785, -0.0164, 0.01787, -0.2049,
        -1.1997, 0, 1.4267, 0, -1.5244, 1.5244, -1.5968,
        -0.0045, 0, 0.4973, 0.9786, 0.00386, -0.01524, -0.2051,
        -1.1997, 0, 1.4267, 0, -1.5244, 1.5244, -1.5968};
    double qvel_zero[32] = {0};

    mju_copy_fp(c->d->qpos, qpos_init, 35);
    mju_copy_fp(c->d->qvel, qvel_zero, c->m->nv);

    c->d->time = 0.0;
    mj_forward_fp(c->m, c->d);
}

float *cassie_sim_geom_rgba(cassie_sim_t *c)
{
    return c->m->geom_rgba;
}

void cassie_sim_set_geom_rgba(cassie_sim_t *c, float *rgba)
{
    for (int i = 0; i < c->m->ngeom * 4; i++)
    {
        c->m->geom_rgba[i] = rgba[i];
    }
}    

double *cassie_sim_geom_quat(cassie_sim_t *c)
{
    return c->m->geom_quat;
}

void cassie_sim_set_geom_quat(cassie_sim_t *c, double *quat)
{
    for (int i = 0; i < c->m->ngeom * 4; i++)
    {
        c->m->geom_quat[i] = quat[i];
    }
}

void cassie_sim_set_geom_name_quat(cassie_sim_t *c, const char* name, double *quat)
{
    int geom_id = mj_name2id_fp(c->m, mjOBJ_GEOM, name);
    mju_copy_fp(&c->m->geom_quat[4 * geom_id], quat, 4);
}

int *cassie_sim_params(cassie_sim_t *c)
{
    int *params = malloc(sizeof(int)*4);
    params[0] = c->m->nq;
    params[1] = c->m->nv;
    params[2] = c->m->nu;
    params[3] = c->m->nsensordata;
    return params;
}

void *cassie_sim_mjmodel(cassie_sim_t *c)
{
    return c->m;
}


void *cassie_sim_mjdata(cassie_sim_t *c)
{
    return c->d;
}


bool cassie_sim_check_obstacle_collision(const cassie_sim_t *c)
{
    for (int i = 0; i < c->d->ncon; ++i) {
        if (c->m->geom_user[c->d->contact[i].geom1] == 1)
            return true;
        if (c->m->geom_user[c->d->contact[i].geom2] == 1)
            return true;
    }

    return false;
}


bool cassie_sim_check_self_collision(const cassie_sim_t *c)
{
    for (int i = 0; i < c->d->ncon; ++i) {
        if (c->m->geom_user[c->d->contact[i].geom1] == 2 &&
            c->m->geom_user[c->d->contact[i].geom2] == 2)
            return true;
    }

    return false;
}

void cassie_sim_foot_positions(const cassie_sim_t *c, double cpos[6])
{
    // Zero the output foot positions 
    mju_zero_fp(cpos, 6);

    for (int i = 0; i < 3; ++i) {
        // Get foot xyz (global coords)
        cpos[i]     = c->d->xpos[3 * left_foot_body_id + i];
        cpos[3 + i] = c->d->xpos[3 * right_foot_body_id + i];
    }

    // cassie mechanical model offset
    // double offset_footJoint2midFoot = sqrt(pow((0.052821 + 0.069746)/2, 2) + pow((0.092622 + 0.010224)/2, 2));
    double offset_footJoint2midFoot = sqrt(pow(0.01762, 2) + pow(0.05219, 2)); // from cassie agility doc
    cpos[2] = cpos[2] - offset_footJoint2midFoot; // foot pos are negative
    cpos[5] = cpos[5] - offset_footJoint2midFoot;
}

void cassie_sim_foot_velocities(const cassie_sim_t *c, double cvel[12])
{
    // Calculate body CoM velocities
    mj_comVel_fp(c->m, c->d);
    // Zero the output foot velocities 
    mju_zero_fp(cvel, 12);

    for (int i = 0; i < 6; ++i) {
        // Get foot xyz (global coords)
        cvel[i]     = c->d->cvel[6 * left_foot_body_id + i];
        cvel[6 + i] = c->d->cvel[6 * right_foot_body_id + i];
    }

}

void cassie_sim_foot_forces(const cassie_sim_t *c, double cfrc[12])
{
    double force_torque[6];
    double force_global[3];

    // Zero the output foot forces
    mju_zero_fp(cfrc, 12);

    // Accumulate the forces on each foot
    for (int i = 0; i < c->d->ncon; ++i) {
        // Get body IDs for both geoms in the collision
        int body1 = c->m->geom_bodyid[c->d->contact[i].geom1];
        int body2 = c->m->geom_bodyid[c->d->contact[i].geom2];

        // Left foot
        if (body1 == left_foot_body_id || body2 == left_foot_body_id) {
            // Get contact force in world coordinates
            mj_contactForce_fp(c->m, c->d, i, force_torque);
            mju_rotVecMatT_fp(force_global, force_torque,
                             c->d->contact[i].frame);

            // Add to total forces on foot
            if (body1 == left_foot_body_id)
                for (int j = 0; j < 3; ++j) cfrc[j] -= force_global[j];
            else
                for (int j = 0; j < 3; ++j) cfrc[j] += force_global[j];
        }

        // Right foot
        if (body1 == right_foot_body_id || body2 == right_foot_body_id) {
            // Get contact force in world coordinates
            mj_contactForce_fp(c->m, c->d, i, force_torque);
            mju_rotVecMatT_fp(force_global, force_torque,
                             c->d->contact[i].frame);

            // Add to total forces on foot
            if (body1 == right_foot_body_id)
                for (int j = 0; j < 3; ++j) cfrc[j+6] -= force_global[j];
            else
                for (int j = 0; j < 3; ++j) cfrc[j+6] += force_global[j];
        }
    }
}

void cassie_vis_foot_forces(const cassie_vis_t *c, double cfrc[12])
{
    double force_torque[6];
    double force_global[3];

    // Zero the output foot forces
    mju_zero_fp(cfrc, 12);

    // Accumulate the forces on each foot
    for (int i = 0; i < c->d->ncon; ++i) {
        // Get body IDs for both geoms in the collision
        int body1 = c->m->geom_bodyid[c->d->contact[i].geom1];
        int body2 = c->m->geom_bodyid[c->d->contact[i].geom2];

        // Left foot
        if (body1 == left_foot_body_id || body2 == left_foot_body_id) {
            // Get contact force in world coordinates
            mj_contactForce_fp(c->m, c->d, i, force_torque);
            mju_rotVecMatT_fp(force_global, force_torque,
                             c->d->contact[i].frame);

            // Add to total forces on foot
            if (body1 == left_foot_body_id)
                for (int j = 0; j < 3; ++j) cfrc[j] -= force_global[j];
            else
                for (int j = 0; j < 3; ++j) cfrc[j] += force_global[j];
        }

        // Right foot
        if (body1 == right_foot_body_id || body2 == right_foot_body_id) {
            // Get contact force in world coordinates
            mj_contactForce_fp(c->m, c->d, i, force_torque);
            mju_rotVecMatT_fp(force_global, force_torque,
                             c->d->contact[i].frame);

            // Add to total forces on foot
            if (body1 == right_foot_body_id)
                for (int j = 0; j < 3; ++j) cfrc[j+6] -= force_global[j];
            else
                for (int j = 0; j < 3; ++j) cfrc[j+6] += force_global[j];
        }
    }
}


void cassie_sim_apply_force(cassie_sim_t *c, double xfrc[6], const char* name)
{
    int body_id = mj_name2id_fp(c->m, mjOBJ_BODY, name);
    mju_copy_fp(&c->d->xfrc_applied[6 * body_id], xfrc, 6);
}


void cassie_sim_clear_forces(cassie_sim_t *c)
{
    mju_zero_fp(c->d->xfrc_applied, 6 * c->m->nbody);
}


void cassie_sim_hold(cassie_sim_t *c)
{
    // Set stiffness/damping for body translation joints
    for (int i = 0; i < 3; ++i) {
        c->m->jnt_stiffness[i] = 1e5;
        c->m->dof_damping[i] = 1e4;
        c->m->qpos_spring[i] = c->d->qpos[i];
    }

    // Set damping for body rotation joint
    for (int i = 3; i < 7; ++i)
        c->m->dof_damping[i] = 1e3;
}


void cassie_sim_release(cassie_sim_t *c)
{
    // Zero stiffness/damping for body translation joints
    for (int i = 0; i < 3; ++i) {
        c->m->jnt_stiffness[i] = 0;
        c->m->dof_damping[i] = 0;
    }

    // Zero damping for body rotation joint
    for (int i = 3; i < 7; ++i)
        c->m->dof_damping[i] = 0;
}


void cassie_sim_radio(cassie_sim_t *c, double channels[16])
{
    for (int i = 0; i < 16; ++i)
        c->cassie_out.pelvis.radio.channel[i] = channels[i];
}

void cassie_sim_full_reset(cassie_sim_t *c)
{
    double qpos_init[35] = {0, 0, 1.01, 1, 0, 0, 0,
        0.0045, 0, 0.4973, 0.9785, -0.0164, 0.01787, -0.2049,
        -1.1997, 0, 1.4267, 0, -1.5244, 1.5244, -1.5968,
        -0.0045, 0, 0.4973, 0.9786, 0.00386, -0.01524, -0.2051,
        -1.1997, 0, 1.4267, 0, -1.5244, 1.5244, -1.5968};

    double ctrl_zero[c->m->nu];
    memset(ctrl_zero, 0, c->m->nu*sizeof(double));
    double xfrc_zero[c->m->nbody*6];
    memset(xfrc_zero, 0, c->m->nbody*sizeof(double));
    mju_copy_fp(c->d->qpos, qpos_init, 35);
    mju_zero_fp(c->d->qvel, c->m->nv);
    mju_zero_fp(c->d->ctrl, c->m->nu);
    mju_zero_fp(c->d->qfrc_applied, c->m->nv);
    mju_zero_fp(c->d->xfrc_applied, 6 * c->m->nbody);
    mju_zero_fp(c->d->qacc, c->m->nv);

    for(int i = 0; i < TORQUE_DELAY_CYCLES; i++) {
        for (int j = 0; j < NUM_DRIVES; j++) {
            c->torque_delay[j][i] = 0;
        }
    }
    state_output_setup(c->estimator);
}

void cassie_vis_full_reset(cassie_vis_t *v)
{
    double qpos_init[35] = {0, 0, 1.01, 1, 0, 0, 0,
        0.0045, 0, 0.4973, 0.9785, -0.0164, 0.01787, -0.2049,
        -1.1997, 0, 1.4267, 0, -1.5244, 1.5244, -1.5968,
        -0.0045, 0, 0.4973, 0.9786, 0.00386, -0.01524, -0.2051,
        -1.1997, 0, 1.4267, 0, -1.5244, 1.5244, -1.5968};

    double qvel_zero[32] = {0};
    double ctrl_zero[v->m->nu];
    memset(ctrl_zero, 0, v->m->nu*sizeof(double));
    double xfrc_zero[v->m->nbody*6];
    memset(xfrc_zero, 0, v->m->nbody*sizeof(double));
    mju_copy_fp(v->d->qpos, qpos_init, 35);
    mju_copy_fp(v->d->qvel, qvel_zero, v->m->nv);
    mju_zero_fp(v->d->ctrl, v->m->nu);
    mju_zero_fp(v->d->qfrc_applied, v->m->nv);
    mju_zero_fp(v->d->xfrc_applied, 6 * v->m->nbody);
    mju_zero_fp(v->d->qacc, v->m->nv);
}

void cassie_vis_apply_force(cassie_vis_t *v, double xfrc[6], const char* name)
{
    int body_id = mj_name2id_fp(v->m, mjOBJ_BODY, name);
    // mju_copy_fp(&v->d->xfrc_applied[6 * body_id], xfrc, 6);
    v->perturb_body = body_id;
    for (int i = 0; i < 6; i++) {
        v->perturb_force[i] = xfrc[i];
    }
}

void scroll(GLFWwindow* window, double xoffset, double yoffset)
{
    (void)xoffset;
    cassie_vis_t* v = glfwGetWindowUserPointer_fp(window);
    // scroll: emulate vertical mouse motion = 5% of window height
    mjv_moveCamera_fp(v->m, mjMOUSE_ZOOM, 0.0, -0.05 * yoffset, &v->scn, &v->cam);
}

void mouse_move(GLFWwindow* w, double xpos, double ypos) {
    cassie_vis_t* v = glfwGetWindowUserPointer_fp(w);

    // no buttons down: nothing to do
    if (!v->button_left && !v->button_middle && !v->button_right) {
        return;
    }

    // compute mouse displacement, save
    double dx = xpos - v->lastx;
    double dy = ypos - v->lasty;
    v->lastx = xpos;
    v->lasty = ypos;

    int width;
    int height;
    glfwGetWindowSize_fp(w, &width, &height);

    int mod_shift = glfwGetKey_fp(w, GLFW_KEY_LEFT_SHIFT) || glfwGetKey_fp(w, GLFW_KEY_RIGHT_SHIFT);

    // determine action based on mouse button
    int action = mjMOUSE_ZOOM;
    if (v->button_right) {
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    } else if (v->button_left) {
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    }

    // move perturb or camera
    mjtNum xchange = dx / height;
    mjtNum ychange = dy / height;
    if (v->pert.active != 0) {
        mjv_movePerturb_fp(v->m, v->d, action, xchange, ychange, &v->scn, &v->pert);
    } else {
        mjv_moveCamera_fp(v->m, action, xchange, ychange, &v->scn, &v->cam);
    }
}

// past data for double-click detection
void mouse_button(GLFWwindow* window, int button, int act, int mods) {
    cassie_vis_t* v = glfwGetWindowUserPointer_fp(window);
    // update button state
    v->button_left = glfwGetMouseButton_fp(window, GLFW_MOUSE_BUTTON_LEFT);
    v->button_middle = glfwGetMouseButton_fp(window, GLFW_MOUSE_BUTTON_MIDDLE);
    v->button_right = glfwGetMouseButton_fp(window, GLFW_MOUSE_BUTTON_RIGHT);

    // Alt: swap left and right
    if (mods == GLFW_MOD_ALT) {
        bool tmp = v->button_left;
        v->button_left = v->button_right;
        v->button_right = tmp;

        if (button == GLFW_MOUSE_BUTTON_LEFT) {
            button = GLFW_MOUSE_BUTTON_RIGHT;
        } else if (button == GLFW_MOUSE_BUTTON_RIGHT) {
            button = GLFW_MOUSE_BUTTON_LEFT;
        }
    }

    // update mouse position
    double x, y;
    glfwGetCursorPos_fp(window, &x, &y);
    v->lastx = x;
    v->lasty = y;

    // set perturbation
    int newperturb = 0;
    if (mods == GLFW_MOD_CONTROL && v->pert.select > 0) {
        if (act == GLFW_PRESS) {
            // Disable vis perturb force when using mouse perturb, only want to vis perturb object
            v->opt.flags[11] = 0;   
            // right: translate;  left: rotate
            if (v->button_right) {
                newperturb = mjPERT_TRANSLATE;
            } else if (v->button_left) {
                newperturb = mjPERT_ROTATE;
            }
            // perturbation onset: reset reference
            if (newperturb > 0 && v->pert.active == 0) {
                mjv_initPerturb_fp(v->m, v->d, &v->scn, &v->pert);
            }
        } else {
            // Enable vis perturb force again
            v->opt.flags[11] = 1;   
        }
    }
    v->pert.active = newperturb;

    // detect double-click (250 msec)
    time_t curr_time = time(0);
    if (act == GLFW_PRESS && (curr_time - v->lastclicktm < 0.25) && (button == v->lastbutton)) {
        // determine selection mode
        int selmode = 2;    // Right Click
        if (button == GLFW_MOUSE_BUTTON_LEFT) {
            selmode = 1;
        } else if (mods == GLFW_MOD_CONTROL) {
            selmode = 3; // CTRL + Right Click
        }
        // get current window size
        int width, height;
        glfwGetWindowSize_fp(window, &width, &height);
        // find geom and 3D click point, get corresponding body
        mjtNum selpnt[3];

        int selgeom = 0;
        int selskin = 0;
        mjtNum aspectratio = (mjtNum) width / height;
        mjtNum relx = (mjtNum) x / width;
        mjtNum rely = (mjtNum) (height - y) / height;

        int selbody = mjv_select_fp(v->m, v->d, &v->opt,
                            aspectratio, relx,
                            rely, 
                            &v->scn, selpnt, &selgeom, &selskin);
        // set lookat point, start tracking is requested
        if (selmode == 2 || selmode == 3) {
            // copy selpnt if geom clicked
            if (selbody >= 0) {
                memcpy(v->cam.lookat, selpnt, sizeof(v->cam.lookat));
            }

            // switch to tracking camera
            if (selmode == 3 && selbody >= 0) {
                v->cam.type = mjCAMERA_TRACKING;
                v->cam.trackbodyid = selbody;
                v->cam.fixedcamid = -1;
            }
        } else { // set body selection
            if (selbody >= 0) {
                // compute localpos
                mjtNum tmp[3];
                mju_sub3_fp(tmp, selpnt, v->d->qpos+3*selbody);
                mju_mulMatTVec_fp(v->pert.localpos, v->d->xmat+9*selbody, tmp, 3, 3);

                // record selection
                v->pert.select = selbody;
                v->pert.skinselect = selskin;
            } else {
                v->pert.select = 0;
                v->pert.skinselect = -1;
            }
        }

        // stop perturbation on select
        v->pert.active = 0;
    }
    // save info
    if (act == GLFW_PRESS) {
        v->lastbutton = button;
        v->lastclicktm = time(0);
    }
    
}

void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    (void)scancode;
    cassie_vis_t* v = glfwGetWindowUserPointer_fp(window);
    if (action == GLFW_RELEASE) {
        return;
    } else if (action == GLFW_PRESS) {
        if (key == GLFW_KEY_P && mods == 0) {
            printf("attaching camera to pelvis\n");
            v->cam.type = mjCAMERA_TRACKING;
            v->cam.trackbodyid = 1;
            v->cam.fixedcamid = -1;
            mjv_moveCamera_fp(v->m, mjMOUSE_ZOOM, 0.0, -0.05*8, &v->scn, &v->cam);
            mjv_moveCamera_fp(v->m, action, 0, -.15, &v->scn, &v->cam);
        }
        // control keys
        if (mods == GLFW_MOD_CONTROL) {
            if (key == GLFW_KEY_A) {
                memcpy(v->cam.lookat, v->m->stat.center, sizeof(v->cam.lookat));
                v->cam.distance = 1.5*v->m->stat.extent;
                // set to free camera
                v->cam.type = mjCAMERA_FREE;
            } else if (key == GLFW_KEY_P) {
                printf("qpos: ");
                for (int i = 0; i < v->m->nq; i++) {
                    printf("%f", v->d->qpos[i]);
                    if (i != v->m->nq-1) {
                        printf(", ");
                    }
                }
                printf("\n");
                // mju_printMat_fp(v->d->qpos, v->m->nq, 1);
            } else if (key == GLFW_KEY_Q) {
                glfwSetWindowShouldClose_fp(window, true);
            }
        }
        // toggle visualiztion flag
        for (int i=0; i < mjNVISFLAG; i++) {
            if (key == VISSTRING[i][2][0]) {
                mjtByte flags[mjNVISFLAG];
                memcpy(flags, v->opt.flags, sizeof(flags));
                flags[i] = flags[i] == 0 ? 1 : 0;
                memcpy(v->opt.flags, flags, sizeof(v->opt.flags));
                return;
            }
        }
        // toggle rendering flag
        for (int i=0; i < mjNRNDFLAG; i++) {
            if (key == RNDSTRING[i][2][0]) {
                mjtByte flags[mjNRNDFLAG];
                memcpy(flags, v->scn.flags, sizeof(flags));
                flags[i] = flags[i] == 0 ? 1 : 0;
                memcpy(v->scn.flags, flags, sizeof(v->scn.flags));
                return;
            }
        }
        // toggle geom/site group
        for (int i=0; i < mjNGROUP; i++) {
            if (key == i + 48) {    // Int('0') = 48
                if (mods && GLFW_MOD_SHIFT == true) {
                    mjtByte sitegroup[mjNGROUP];
                    memcpy(sitegroup, v->opt.sitegroup, sizeof(sitegroup));
                    sitegroup[i] = sitegroup[i] > 0 ? 0 : 1;
                    // memcpy(v->opt.sitegroup = sitegroup
                    v->opt.sitegroup[i] = sitegroup[i];
                    return;
                } else {
                    mjtByte geomgroup[mjNGROUP];
                    memcpy(geomgroup, v->opt.geomgroup, sizeof(geomgroup));
                    geomgroup[i] = geomgroup[i] > 0 ? 0 : 1;
                    memcpy(v->opt.geomgroup, geomgroup, sizeof(v->opt.geomgroup));
                    return;
                }
            }
        }
        switch (key) {
            case GLFW_KEY_F1: {     // help
                v->showhelp += 1;
                if (v->showhelp > 1) {
                    v->showhelp = 0;
                }
            } break;
            case GLFW_KEY_F2: {     // option
                v->showoption = !v->showoption;
            } break;
            case GLFW_KEY_F3: {     // info
                v->showinfo = !v->showinfo;
            } break;
            case GLFW_KEY_F4: {     // GRF
                v->showGRF = !v->showGRF;
            } break;
            case GLFW_KEY_F5: {     // toggle fullscreen
                v->showfullscreen = !v->showfullscreen;
                v->showfullscreen ? glfwMaximizeWindow_fp(window) : glfwRestoreWindow_fp(window);
            } break;
            case GLFW_KEY_F7: {     // sensor figure
                v->showsensor = !v->showsensor;
            } break;
            case GLFW_KEY_ENTER: {  // slow motion
                v->slowmotion = !v->slowmotion;
                v->slowmotion ? printf("Slow Motion Mode!\n") : printf("Normal Speed Mode!\n");
            } break;
            case GLFW_KEY_SPACE: {  // pause
                v->paused = !v->paused;
                v->paused ? printf("Paused\n") : printf("Running\n");
            } break;
            case GLFW_KEY_BACKSPACE: {  // reset
                double qpos_init[35] =
                    {0, 0, 1.01, 1, 0, 0, 0,
                    0.0045, 0, 0.4973, 0.9785, -0.0164, 0.01787, -0.2049,
                    -1.1997, 0, 1.4267, 0, -1.5244, 1.5244, -1.5968,
                    -0.0045, 0, 0.4973, 0.9786, 0.00386, -0.01524, -0.2051,
                    -1.1997, 0, 1.4267, 0, -1.5244, 1.5244, -1.5968};
                double qvel_zero[32] = {0};
                mju_copy_fp(v->d->qpos, qpos_init, 35);
                mju_copy_fp(v->d->qvel, qvel_zero, v->m->nv);
                v->d->time = 0.0;
                mj_forward_fp(v->m, v->d);
            } break;
            case GLFW_KEY_RIGHT: {      // step forward
                if (v->paused) {
                    mj_step_fp(v->m, v->d);
                }
            } break;
            // case GLFW_KEY_LEFT: {       // step backward
            //     if (v->paused) {
            //         double dt = v->m->opt.timestep;
            //         v->m->opt.timestep = -dt;
            //         mj_step_fp(v->m, v->d);
            //         v->m->opt.timestep = dt;
            //     }
            // } break;
            case GLFW_KEY_DOWN: {      // step forward 100
                if (v->paused) {
                    for (int i = 0; i < 100; i++) {
                        mj_step_fp(v->m, v->d);
                    }
                }
            } break;
            // case GLFW_KEY_UP: {       // step back 100
            //     if (v->paused) {
            //         double dt = v->m->opt.timestep;
            //         v->m->opt.timestep = -dt;
            //         for (int i = 0; i < 100; i++) {
            //             mj_step_fp(v->m, v->d);
            //         }
            //         v->m->opt.timestep = dt;
            //     }
            // } break;
            case GLFW_KEY_ESCAPE: {     // free camera
                v->cam.type = mjCAMERA_FREE;
            } break;
            case GLFW_KEY_EQUAL: {      // bigger font
                if (fontscale < 200) {
                    fontscale += 50;
                    mjr_makeContext_fp(v->m, &v->con, fontscale);
                }
            } break;
            case GLFW_KEY_MINUS: {      // smaller font
                if (fontscale > 100) {
                    fontscale -= 50;
                    mjr_makeContext_fp(v->m, &v->con, fontscale);
                }
            } break;
            case GLFW_KEY_LEFT_BRACKET: {  // '[' previous fixed camera or free
                int fixedcam = v->cam.type;
                if (v->m->ncam > 0 && fixedcam == mjCAMERA_FIXED) {
                    int fixedcamid = v->cam.fixedcamid;
                    if (fixedcamid  > 0) {
                        v->cam.fixedcamid = fixedcamid-1;
                    } else {
                        v->cam.type = mjCAMERA_FREE;
                    }
                }
            } break;
            case GLFW_KEY_RIGHT_BRACKET: {  // ']' next fixed camera
                if (v->m->ncam > 0) {
                    int fixedcam = v->cam.type;
                    int fixedcamid = v->cam.fixedcamid;
                    if (fixedcam != mjCAMERA_FIXED) {
                        v->cam.type = mjCAMERA_FIXED;
                    } else if (fixedcamid < v->m->ncam - 1) {
                        v->cam.fixedcamid = fixedcamid+1;
                    }
                }
            } break;
        }
    }

}

void sensorinit(cassie_vis_t *v) {
    mjv_defaultFigure_fp(&v->figsensor);
    v->figsensor.figurergba[3] = 0.5f;

    // Set flags
    v->figsensor.flg_extend = 1;
    v->figsensor.flg_barplot = 1;
    v->figsensor.flg_symmetric = 1;

    strcpy(v->figsensor.title, "Sensor data");
    
    // y-tick number format
    strcpy(v->figsensor.yformat, "%.0f");
    // grid size
    v->figsensor.gridsize[0] = 2;
    v->figsensor.gridsize[1] = 3;
    // minimum range
    v->figsensor.range[0][0] = 0;
    v->figsensor.range[0][1] = 0;
    v->figsensor.range[1][0] = -1;
    v->figsensor.range[1][1] = 1;
    // int min_range[2][2] = { {0, 1}, {-1, 1} };
    // memcpy(min_range, v->figsensor.range, sizeof(min_range));

}

// update sensor figure
void sensorupdate(cassie_vis_t* v) {
    static const int maxline = 10;

    // clear linepnt
    for( int i=0; i<maxline; i++ )
        v->figsensor.linepnt[i] = 0;

    // start with line 0
    int lineid = 0;

    // loop over sensors
    for( int n=0; n<v->m->nsensor; n++ )
    {
        // go to next line if type is different
        if( n>0 && v->m->sensor_type[n]!=v->m->sensor_type[n-1] )
            lineid = mjMIN(lineid+1, maxline-1);

        // get info about this sensor
        mjtNum cutoff = (v->m->sensor_cutoff[n]>0 ? v->m->sensor_cutoff[n] : 1);
        int adr = v->m->sensor_adr[n];
        int dim = v->m->sensor_dim[n];

        // data pointer in line
        int p = v->figsensor.linepnt[lineid];

        // fill in data for this sensor
        for( int i=0; i<dim; i++ )
        {
            // check size
            if( (p+2*i)>=mjMAXLINEPNT/2 )
                break;

            // x
            v->figsensor.linedata[lineid][2*p+4*i] = (float)(adr+i);
            v->figsensor.linedata[lineid][2*p+4*i+2] = (float)(adr+i);

            // y
            v->figsensor.linedata[lineid][2*p+4*i+1] = 0;
            v->figsensor.linedata[lineid][2*p+4*i+3] = (float)(v->d->sensordata[adr+i]/cutoff);
        }

        // update linepnt
        v->figsensor.linepnt[lineid] = mjMIN(mjMAXLINEPNT-1, 
                                          figsensor.linepnt[lineid]+2*dim);
    }
}

// show sensor figure
void sensorshow(cassie_vis_t* v, mjrRect rect) {
    // constant width with and without profiler
    int width = rect.width/4;

    // render figure on the right
    mjrRect viewport = {
        rect.left + 3*width, 
        rect.bottom, 
        width, 
        rect.height/3
    };
    mjr_figure_fp(viewport, &v->figsensor, &v->con);
}

void grfinit(cassie_vis_t *v) {
    mjv_defaultFigure_fp(&v->figGRF);
    v->figGRF.figurergba[3] = 0.5f;

    // Set flags
    // v->figGRF.flg_extend = 1;
    // v->figGRF.flg_barplot = 1;

    strcpy(v->figGRF.title, "Ground Reaction Forces");
    
    // y-tick number format
    strcpy(v->figGRF.yformat, "%.2f");
    // grid size
    v->figGRF.gridsize[0] = 5;
    v->figGRF.gridsize[1] = 5;
    // minimum range
    v->figGRF.range[0][0] = -200;
    v->figGRF.range[0][1] = 0;
    v->figGRF.range[1][0] = 0;
    v->figGRF.range[1][1] = 20;
    // legends
    strcpy(v->figGRF.linename[0], "left foot");
    strcpy(v->figGRF.linename[1], "right foot");

    // init x axis (don't show yet)
    for(int n=0; n < 2; n++) {
        for(int i = 0; i < mjMAXLINEPNT; i++) {
            v->figGRF.linedata[n][2*i] = (float)-i;
        }
    }
    // int min_range[2][2] = { {0, 1}, {-1, 1} };
    // memcpy(min_range, v->figsensor.range, sizeof(min_range));

}

void grfupdate(cassie_vis_t* v) {
    // if (v->GRFcount == 0) {
        double cfrc[12];
        cassie_vis_foot_forces(v, cfrc);
        float tdata[2] = { 
            (float)(cfrc[2]),
            (float)(cfrc[8])
        };

        // update figtimer
        int pnt = mjMIN(201, v->figGRF.linepnt[0]+1);
        for(int n = 0; n < 2; n++) {
            // shift data
            for(int i = pnt - 1; i > 0; i--) {
                v->figGRF.linedata[n][2*i+1] = v->figGRF.linedata[n][2*i-1];
            }
            // assign new
            v->figGRF.linepnt[n] = pnt;
            v->figGRF.linedata[n][1] = tdata[n];
        }
//     } 
//     v->GRFcount ++;
//     if (v->GRFcount > 9) {
//         v->GRFcount = 0;
//     }
}

// show sensor figure
void grfshow(cassie_vis_t* v, mjrRect rect) {
    mjrRect viewport = {
        rect.left + rect.width - rect.width/2, 
        rect.bottom + rect.height/3, 
        rect.width/2, 
        rect.height/2
    };

    mjr_figure_fp(viewport, &v->figGRF, &v->con);
}

cassie_vis_t *cassie_vis_init(cassie_sim_t* c, const char* modelfile) {
    // Make sure MuJoCo is initialized and the model is loaded
    if (!mujoco_initialized) {
        printf("vis mujoco not init\n");
        if (!cassie_mujoco_init(modelfile)) {
            printf("mujoco not init\n");
            return NULL;
        }
    }

    if (!glfw_initialized) {
        printf("glfw not init\n");
        return NULL;
    }
    // Allocate visualization structure
    cassie_vis_t *v = malloc(sizeof (cassie_vis_t));
    // Set interaction ctrl vars
    v->lastx = 0.0;
    v->lasty = 0.0;
    v->button_left = false;
    v->button_middle = false;
    v->button_right = false;
    v->lastbutton = GLFW_MOUSE_BUTTON_1;
    v->lastclicktm = 0.0;
    // GLFWvidmode* vidmode = glfwGetVideoMode_fp(glfwGetPrimaryMonitor_fp());
    v->refreshrate = glfwGetVideoMode_fp(glfwGetPrimaryMonitor_fp())->refreshRate;
    v->showhelp = 0;
    v->showoption = false;
    v->showGRF = false;
    v->GRFcount = 0;
    v->showfullscreen = false;
    v->showsensor = false;
    v->slowmotion = false;
    v->showinfo = true;
    v->paused = true;
    v->framenum = 0;
    v->lastframenum = 0;
    v->m = c->m;
    v->d = c->d;
    v->perturb_body = 1;
    memset(v->perturb_force, 0.0, 6*sizeof(double));

    // Create window
    v->window = glfwCreateWindow_fp(1200, 900, "Cassie", NULL, NULL);
    glfwMakeContextCurrent_fp(v->window);
    glfwSwapInterval_fp(0);

    //printf("Refresh Rate: %i\n", v->refreshrate);
    //printf("Resolution: %ix%i\n", 1200, 900);

    sensorinit(v);
    grfinit(v);
    // Set up mujoco visualization objects
    // v->cam.type = mjCAMERA_FIXED;
    // v->cam.fixedcamid = 0;
    mjv_defaultCamera_fp(&v->cam);
    mjv_defaultOption_fp(&v->opt);
    v->opt.flags[11] = 1;//v->opt.flags[12];    // Render applied forces
    mjr_defaultContext_fp(&v->con);
    mjv_defaultScene_fp(&v->scn);
    mjv_makeScene_fp(c->m, &v->scn, 1000);
    mjr_makeContext_fp(c->m, &v->con, fontscale);

    // Set callback for user-initiated window close events
    glfwSetWindowUserPointer_fp(v->window, v);
    glfwSetWindowCloseCallback_fp(v->window, window_close_callback);

    // Set glfw callbacks
    glfwSetCursorPosCallback_fp(v->window, mouse_move);
    glfwSetMouseButtonCallback_fp(v->window, mouse_button);
    glfwSetScrollCallback_fp(v->window, scroll);
    glfwSetKeyCallback_fp(v->window, key_callback);

    return v;
}


void cassie_vis_close(cassie_vis_t *v)
{
    if (!glfw_initialized || !v || !v->window)
        return;

    // Free mujoco objects
    mjv_freeScene_fp(&v->scn);
    mjr_freeContext_fp(&v->con);

    // Close window
    glfwDestroyWindow_fp(v->window);
    v->window = NULL;
}


void cassie_vis_free(cassie_vis_t *v)
{
    if (!glfw_initialized || !v)
        return;

    // Close the window, if it hasn't been closed already
    if (v->window)
        cassie_vis_close(v);

    // Free cassie_vis_t
    free(v);
}


bool cassie_vis_draw(cassie_vis_t *v, cassie_sim_t *c)
{
    (void)c;
    if (!glfw_initialized)
        return false;

    // Return early if window is closed
    if (!v || !v->window)
        return false;

    // Check if window should be closed
    if (glfwWindowShouldClose_fp(v->window)) {
        cassie_vis_close(v);
        return false;
    }
    // clear old perturbations, apply new
    mju_zero_fp(v->d->xfrc_applied, 6 * v->m->nbody);
    if (v->pert.select > 0) {
       mjv_applyPerturbPose_fp(v->m, v->d, &v->pert, 0); // move mocap bodies only
       mjv_applyPerturbForce_fp(v->m, v->d, &v->pert);
    }
    // Add applied forces to qfrc array
    for (int i = 0; i < 6; i++) {
        v->d->xfrc_applied[6*v->perturb_body + i] += v->perturb_force[i];
    }
    mj_forward_fp(v->m, v->d);
    // Reset xfrc applied to zero
    memset(v->perturb_force, 0, 6*sizeof(double));
    // Set up for rendering
    glfwMakeContextCurrent_fp(v->window);
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize_fp(v->window, &viewport.width, &viewport.height);
    mjrRect smallrect = viewport;
    // Render scene

    mjv_updateScene_fp(v->m, v->d, &v->opt, &v->pert, &v->cam, mjCAT_ALL, &v->scn);
    mjr_render_fp(viewport, &v->scn, &v->con);
    if (v->showsensor) {
        if (!v->paused) {
            sensorupdate(v);
        }
        sensorshow(v, smallrect);
    }
    if (v->showGRF) {
        if (!v->paused) {
            grfupdate(v);
        }
        grfshow(v, smallrect);
    }
    if (v->showhelp) {
        mjr_overlay_fp(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport, help_title, help_content, &v->con);
    }
    if (v->showinfo) {
        char buf[1024];
        char str_slow[20];
        if (v->slowmotion) {
            strcpy(str_slow, "(10x slowdown)");
        } else {
            strcpy(str_slow, "");
        }
        char str_paused[50];
        if(v->paused) {
            strcpy(str_paused, "\nPaused");
        } else {
            strcpy(str_paused, "\nRunning");
        }
        strcat(str_paused, "\nTime:");
        char status[50];
        sprintf(status, "\n\n%.2f", v->d->time);
        strcpy(buf, str_slow);
        strcat(buf, status);
        // status = str_slow * status

        mjr_overlay_fp(mjFONT_NORMAL, mjGRID_BOTTOMLEFT, viewport,
                    str_paused,
                    buf, &v->con);
    }
    // Show updated scene
    glfwSwapBuffers_fp(v->window);
    glfwPollEvents_fp();

    //double cfrc[12];
    // cassie_sim_foot_forces(c, cfrc);
    // printf("foot forces:\n");
    // for (int i = 0; i < 12; i++) {
    //     printf("%f, ", cfrc[i]);
    // }
    return true;
}


bool cassie_vis_valid(cassie_vis_t *v)
{
    if (!glfw_initialized)
        return false;

    return v && v->window;
}

bool cassie_vis_paused(cassie_vis_t *v)
{
    return v->paused;
}

bool cassie_vis_slowmo(cassie_vis_t *v)
{
    return v->slowmotion;
}

cassie_state_t *cassie_state_alloc()
{
    cassie_state_t *s = malloc(sizeof (cassie_state_t));
    CASSIE_ALLOC_POINTER(s);
    return s;
}


cassie_state_t *cassie_state_duplicate(const cassie_state_t *src)
{
    // Allocate new cassie_state_t
    cassie_state_t *s = cassie_state_alloc();

    // Copy data
    cassie_state_copy(s, src);

    return s;
}


void cassie_state_copy(cassie_state_t *dst, const cassie_state_t *src)
{
    // Copy POD types
    CASSIE_COPY_POD(dst, src);

    // Copy pointer types
    CASSIE_COPY_POINTER(dst, src);
}


void cassie_state_free(cassie_state_t *s)
{
    CASSIE_FREE_POINTER(s);
    free(s);
}


double *cassie_state_time(cassie_state_t *s)
{
    return &s->d->time;
}


double *cassie_state_qpos(cassie_state_t *s)
{
    return s->d->qpos;
}


double *cassie_state_qvel(cassie_state_t *s)
{
    return s->d->qvel;
}


void cassie_get_state(const cassie_sim_t *c, cassie_state_t *s)
{
    // Copy POD types
    CASSIE_COPY_POD(s, c);

    // Copy pointer types
    CASSIE_COPY_POINTER(s, c);
}


void cassie_set_state(cassie_sim_t *c, const cassie_state_t *s)
{
    // Copy POD types
    CASSIE_COPY_POD(c, s);

    // Copy pointer types
    CASSIE_COPY_POINTER(c, s);
}
