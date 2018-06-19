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
#include "mujoco.h"
#include "glfw3.h"
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
    X(mj_forward)                               \
    X(mj_step1)                                 \
    X(mj_step2)                                 \
    X(mj_contactForce)                          \
    X(mj_name2id)                               \
    X(mju_copy)                                 \
    X(mju_zero)                                 \
    X(mju_rotVecMatT)                           \
    X(mjv_makeScene)                            \
    X(mjv_freeScene)                            \
    X(mjv_updateScene)                          \
    X(mjv_defaultOption)                        \
    X(mjr_defaultContext)                       \
    X(mjr_makeContext)                          \
    X(mjr_freeContext)                          \
    X(mjr_render)

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
    X(glfwGetFramebufferSize)                   \
    X(glfwSwapBuffers)                          \
    X(glfwSwapInterval)                         \
    X(glfwPollEvents)

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
#define MJLIBNAME "libmujoco150.so"
#define MJLIBNAMENOGL "libmujoco150nogl.so"
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
    GLFWwindow *window;
    mjvCamera cam;
    mjvOption opt;
    mjvScene scn;
    mjrContext con;
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

#define CASSIE_ALLOC_POINTER(c)                 \
    do {                                        \
        c->d = mj_makeData_fp(initial_model);   \
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
    snprintf(buf, sizeof buf, "%.4096s/mjpro150/bin/libglew.so", basedir);
    glew_handle = LOADLIB(buf);
    if (!gl_handle || !glew_handle)
        return false;
#endif

    // Open library
    snprintf(buf, sizeof buf, "%.4096s/mjpro150/bin/" GLFWLIBNAME, basedir);
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


static bool load_mujoco_library(const char *basedir)
{
    // Buffer for paths
    char buf[4096 + 1024];

    // Try loading GLFW
    bool __attribute__((unused)) gl = load_glfw_library(basedir);

    // Choose library version
    snprintf(buf, sizeof buf, "%.4096s/mjpro150/bin/" MJLIBNAME, basedir);
#ifndef _WIN32
    if (!gl)
        snprintf(buf, sizeof buf, "%.4096s/mjpro150/bin/" MJLIBNAMENOGL, basedir);
#endif

    // Open library
    mj_handle = LOADLIB(buf);
    if (!mj_handle) {
        fprintf(stderr, "Failed to load %s\n", buf);
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

#define ID_NAME_LOOKUP(idvar, objtype, name)                            \
    do {                                                                \
        idvar = mj_name2id_fp(initial_model, objtype, #name);           \
        if (-1 == idvar) {                                              \
            fprintf(stderr, "Could not find body named " #name "\n");   \
            return false;                                               \
        }                                                               \
    } while (0)


bool cassie_mujoco_init(const char *basedir)
{
    // Buffer for paths
    char buf[4096 + 1024];

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
        if (!basedir)
            basedir = dirname(binpath);
#endif

        // Load MuJoCo
        if (!load_mujoco_library(basedir))
            return false;

        // Activate MuJoCo
        snprintf(buf, sizeof buf, "%.4096s/mjkey.txt", basedir);
        mj_activate_fp(buf);

        // Load the model
        snprintf(buf, sizeof buf, "%.4096s/cassie.xml", basedir);
        char error[1000] = "Could not load XML model";
        initial_model = mj_loadXML_fp(buf, 0, error, 1000);

        if (!initial_model) {
            fprintf(stderr, "Load model error: %s\n", error);
            return false;
        }

        // Look up relevant IDs based on names
        ID_NAME_LOOKUP(left_foot_body_id, mjOBJ_BODY, left-foot);
        ID_NAME_LOOKUP(right_foot_body_id, mjOBJ_BODY, right-foot);

        mujoco_initialized = true;
    }

    // Initialize GLFW if it was loaded
    if (glfw_handle && !glfw_initialized) {
        if (!glfwInit_fp()) {
            fprintf(stderr, "Could not initialize GLFW\n");
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


cassie_sim_t *cassie_sim_init(void)
{
    // Make sure MuJoCo is initialized and the model is loaded
    if (!mujoco_initialized) {
        if (!cassie_mujoco_init(NULL))
            return NULL;
    }

    // Allocate memory, zeroed for cassie_out_t and filter initialization
    cassie_sim_t *c = calloc(1, sizeof (cassie_sim_t));

    // Initialize cassie outputs
    cassie_out_init(&c->cassie_out);

    // Filters initialized to zero

    // Initialize mjModel
    c->m = mj_copyModel_fp(NULL, initial_model);

    // Allocate pointer types
    CASSIE_ALLOC_POINTER(c);

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
    CASSIE_ALLOC_POINTER(c);

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
    CASSIE_COPY_POINTER(dst, src);
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


void cassie_sim_apply_force(cassie_sim_t *c, double xfrc[6], int body)
{
    mju_copy_fp(&c->d->xfrc_applied[6 * body], xfrc, 6);
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


cassie_vis_t *cassie_vis_init()
{
    // Make sure MuJoCo is initialized and the model is loaded
    if (!mujoco_initialized) {
        if (!cassie_mujoco_init(NULL))
            return NULL;
    }

    if (!glfw_initialized)
        return NULL;

    // Allocate visualization structure
    cassie_vis_t *v = malloc(sizeof (cassie_vis_t));

    // Create window
    v->window = glfwCreateWindow_fp(1200, 900, "Cassie", NULL, NULL);
    glfwMakeContextCurrent_fp(v->window);
    glfwSwapInterval_fp(0);

    // Set up mujoco visualization objects
    v->cam.type = mjCAMERA_FIXED;
    v->cam.fixedcamid = 0;
    mjv_defaultOption_fp(&v->opt);
    mjr_defaultContext_fp(&v->con);
    mjv_makeScene_fp(&v->scn, 1000);
    mjr_makeContext_fp(initial_model, &v->con, mjFONTSCALE_100);

    // Set callback for user-initiated window close events
    glfwSetWindowUserPointer_fp(v->window, v);
    glfwSetWindowCloseCallback_fp(v->window, window_close_callback);

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
    if (!glfw_initialized)
        return false;

    // Return early if window is closed
    if (!v || !v->window)
        return false;

    // Set up for rendering
    glfwMakeContextCurrent_fp(v->window);
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize_fp(v->window, &viewport.width, &viewport.height);

    // Render scene
    mjv_updateScene_fp(c->m, c->d, &v->opt, NULL, &v->cam, mjCAT_ALL, &v->scn);
    mjr_render_fp(viewport, &v->scn, &v->con);

    // Show updated scene
    glfwSwapBuffers_fp(v->window);
    glfwPollEvents_fp();

    return true;
}


bool cassie_vis_valid(cassie_vis_t *v)
{
    if (!glfw_initialized)
        return false;

    return v && v->window;
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
