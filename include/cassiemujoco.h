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

#ifndef CASSIEMUJOCO_H
#define CASSIEMUJOCO_H

#include <stdbool.h>
#include "cassie_out_t.h"
#include "cassie_in_t.h"
#include "cassie_user_in_t.h"
#include "state_out_t.h"
#include "pd_in_t.h"


typedef struct cassie_sim cassie_sim_t;
typedef struct cassie_vis cassie_vis_t;
typedef struct cassie_state cassie_state_t;


#ifdef __cplusplus
extern "C" {
#endif

// Pass a null-terminated string containing the path to the directory
// containing cassie.xml, mjpro150/, mjkey.txt, etc. If NULL is
// passed, the directory containing the current executable is used
// instead. Returns true if loading was successful, false otherwise.
bool cassie_mujoco_init(const char *modelfile);

// Unloads the MuJoCo library and Cassie model. After calling this
// function, cassie_mujoco_init can be called again.
void cassie_cleanup(void);


/*******************************************************************************
 * Cassie simulator functions
 ******************************************************************************/

// Creates an instance of the Cassie simulator. If called before
// cassie_mujoco_init, cassie_mujoco_init is called with the parameter
// NULL.
cassie_sim_t *cassie_sim_init(const char *modelfile);

// Creates an instance of the Cassie simulator with the same state as
// an existing instance.
cassie_sim_t *cassie_sim_duplicate(const cassie_sim_t *sim);

// Copies the state of one Cassie simulator to another.
void cassie_sim_copy(cassie_sim_t *dst, const cassie_sim_t *src);

// Destroys an instance of the Cassie simulator.
void cassie_sim_free(cassie_sim_t *sim);

// Simulates one step of the Cassie simulator at the lowest level of
// input and output. Only one cassie_sim_step_* function should be
// called on a given Cassie simulator instance.
void cassie_sim_step_ethercat(cassie_sim_t *sim, cassie_out_t *y, const cassie_in_t *u);

// Simulates one step of the Cassie simulator including software
// safeties. Only one cassie_sim_step_* function should be called on a
// given Cassie simulator instance.
void cassie_sim_step(cassie_sim_t *sim, cassie_out_t *y, const cassie_user_in_t *u);

// Simulates one step of the Cassie simulator with PD input and state
// estimator output. Only one cassie_sim_step_* function should be
// called on a given Cassie simulator instance.
void cassie_sim_step_pd(cassie_sim_t *sim, state_out_t *y, const pd_in_t *u);

// Returns a read-write pointer to the simulator time.
double *cassie_sim_time(cassie_sim_t *sim);

// Returns a read-write pointer to the simulator joint positions.
// The order of the values are as follows:
// [ 0] Pelvis x
// [ 1] Pelvis y
// [ 2] Pelvis z
// [ 3] Pelvis orientation qw
// [ 4] Pelvis orientation qx
// [ 5] Pelvis orientation qy
// [ 6] Pelvis orientation qz
// [ 7] Left hip roll         (Motor [0])
// [ 8] Left hip yaw          (Motor [1])
// [ 9] Left hip pitch        (Motor [2])
// [10] Left achilles rod qw
// [11] Left achilles rod qx
// [12] Left achilles rod qy
// [13] Left achilles rod qz
// [14] Left knee             (Motor [3])
// [15] Left shin                        (Joint [0])
// [16] Left tarsus                      (Joint [1])
// [17] Left heel spring
// [18] Left foot crank
// [19] Left plantar rod
// [20] Left foot             (Motor [4], Joint [2])
// [21] Right hip roll        (Motor [5])
// [22] Right hip yaw         (Motor [6])
// [23] Right hip pitch       (Motor [7])
// [24] Right achilles rod qw
// [25] Right achilles rod qx
// [26] Right achilles rod qy
// [27] Right achilles rod qz
// [28] Right knee            (Motor [8])
// [29] Right shin                       (Joint [3])
// [30] Right tarsus                     (Joint [4])
// [31] Right heel spring
// [32] Right foot crank
// [33] Right plantar rod
// [34] Right foot            (Motor [9], Joint [5])
double *cassie_sim_qpos(cassie_sim_t *sim);

// Returns a read-write pointer to the simulator joint velocities.
// The order of the values are as follows:
// [ 0] Pelvis x
// [ 1] Pelvis y
// [ 2] Pelvis z
// [ 3] Pelvis orientation wx
// [ 4] Pelvis orientation wy
// [ 5] Pelvis orientation wz
// [ 6] Left hip roll         (Motor [0])
// [ 7] Left hip yaw          (Motor [1])
// [ 8] Left hip pitch        (Motor [2])
// [ 9] Left achilles rod wx
// [10] Left achilles rod wy
// [11] Left achilles rod wz
// [12] Left knee             (Motor [3])
// [13] Left shin                        (Joint [0])
// [14] Left tarsus                      (Joint [1])
// [15] Left heel spring
// [16] Left foot crank
// [17] Left plantar rod
// [18] Left foot             (Motor [4], Joint [2])
// [19] Right hip roll        (Motor [5])
// [20] Right hip yaw         (Motor [6])
// [21] Right hip pitch       (Motor [7])
// [22] Right achilles rod wx
// [23] Right achilles rod wy
// [24] Right achilles rod wz
// [25] Right knee            (Motor [8])
// [26] Right shin                       (Joint [3])
// [27] Right tarsus                     (Joint [4])
// [28] Right heel spring
// [29] Right foot crank
// [30] Right plantar rod
// [31] Right foot            (Motor [9], Joint [5])
double *cassie_sim_qvel(cassie_sim_t *sim);

double *cassie_sim_qacc(cassie_sim_t *c);

// Returns the mjModel* used by the simulator
void *cassie_sim_mjmodel(cassie_sim_t *sim);

// Returns the mjData* used by the simulator
void *cassie_sim_mjdata(cassie_sim_t *sim);

// Returns true if any of the collision bodies in Cassie are in
// contact with an object with the obstacle class.
bool cassie_sim_check_obstacle_collision(const cassie_sim_t *sim);

// Returns true if any of the collision bodies in Cassie are in
// contact with each other (i.e. right and left leg collide).
bool cassie_sim_check_self_collision(const cassie_sim_t *sim);

// Returns the contact forces on the left and right feet
// cfrc[0-2]:  Contact force acting on the left foot, in world coordinates
// cfrc[3-5]:  Currently zero, reserved for torque acting on the left foot
// cfrc[6-8]:  Contact force acting on the left foot, in world coordinates
// cfrc[9-11]: Currently zero, reserved for torque acting on the right foot
void cassie_sim_foot_forces(const cassie_sim_t *c, double cfrc[12]);

// Applies an external force to a specified body.
void cassie_sim_apply_force(cassie_sim_t *sim, double xfrc[6], const char* name);

// Sets all external forces to zero.
void cassie_sim_clear_forces(cassie_sim_t *sim);

// Holds the pelvis stationary in the current position.
void cassie_sim_hold(cassie_sim_t *sim);

// Releases a held pelvis.
void cassie_sim_release(cassie_sim_t *sim);

// Sets the values reported by the radio receiver in Cassie, which
// should be doubles in the range [-1, 1]. Channel 8 must be set to 1
// to enable the motors, which is the default state.
void cassie_sim_radio(cassie_sim_t *sim, double channels[16]);

// Does a "full reset", i.e. sets qpos to a starting position and zeros
// out all other data used for computation (like velocities, accelerations, forces)
void cassie_sim_full_reset(cassie_sim_t *sim);

double* cassie_sim_xquat(cassie_sim_t *c, const char* name);

/*******************************************************************************
 * Cassie visualizer functions
 ******************************************************************************/

// Creates an instance of the Cassie simulation visualizer. If called
// before cassie_mujoco_init, cassie_mujoco_init is called with the
// parameter NULL.
cassie_vis_t *cassie_vis_init(cassie_sim_t *sim, const char* modelfile);

// Closes the visualization window without freeing the instance. After
// calling this, cassie_vis_draw can still be called, but the
// visualizer will remain closed.
void cassie_vis_close(cassie_vis_t *vis);

// Closes and frees the visualization window.
void cassie_vis_free(cassie_vis_t *vis);

// Visualizes the state of the given Cassie simulator.
bool cassie_vis_draw(cassie_vis_t *vis, cassie_sim_t *sim);

// Returns true if the visualizer has been closed but not freed.
bool cassie_vis_valid(cassie_vis_t *vis);

// Returns value of vis->paused
bool cassie_vis_paused(cassie_vis_t *vis);

// Returns value of vis->slowmotion
bool cassie_vis_slowmo(cassie_vis_t *vis);

// Apply inputted perturbation to any body in the vis's mjData
void cassie_vis_apply_force(cassie_vis_t *vis, double xfrc[6], const char* name);

// Does a "full reset", i.e. sets qpos to a starting position and zeros
// out all other data used for computation (like velocities, accelerations, forces)
void cassie_vis_full_reset(cassie_vis_t *sim);

/*******************************************************************************
 * Cassie simulation state functions
 ******************************************************************************/

// Allocates storage for a Cassie simulation state object. This allows
// the state of a simulator to be recorded and restored without
// duplicating the entire simulator. A simulation state can only be
// restored to the exact simulator instance it was recorded from.
cassie_state_t *cassie_state_alloc(void);

// Creates an instance of a simulation state object with the same
// state as an existing instance.
cassie_state_t *cassie_state_duplicate(const cassie_state_t *src);

// Copies the state of one simulation state object into another.
void cassie_state_copy(cassie_state_t *dst, const cassie_state_t *src);

// Destroys a Cassie simulation state object
void cassie_state_free(cassie_state_t *state);

// Returns a read/write pointer to the simulation state time.
double *cassie_state_time(cassie_state_t *state);

// Returns a read/write pointer to the simulation state joint positions.
double *cassie_state_qpos(cassie_state_t *state);

// Returns a read/write pointer to the simulation state joint velocities.
double *cassie_state_qvel(cassie_state_t *state);

// Copies the state of a Cassie simulator into a simulation state object.
void cassie_get_state(const cassie_sim_t *sim, cassie_state_t *state);

// Copies the state of a simulation state object into a Cassie simulator.
void cassie_set_state(cassie_sim_t *sim, const cassie_state_t *state);


#ifdef __cplusplus
}
#endif

#endif // CASSIEMUJOCO_H
