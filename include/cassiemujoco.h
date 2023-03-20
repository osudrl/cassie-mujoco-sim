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
#include <stdio.h>
#include <stdlib.h>
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

cassie_out_t cassie_sim_get_cassie_out(cassie_sim_t *c);

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

// Reloads the xml file used for the cassie mujoco model. Overwrites the
// previously used model. Returns whether successful or not. This is used for overwriting
// the global mjModel that all new cassie sim objects will use by default (reinit set to false)
bool cassie_reload_xml(const char *modelfile);

// Creates an instance of the Cassie simulator. If called before
// cassie_mujoco_init, cassie_mujoco_init is called with the parameter
// NULL. The "reinit" arg allows for the created cassie sim object to use a
// different mjModel than the global one loaded by cassie_mujoco_init. If reinit
// is true, then a new mjModel is made using the inputted "modelfile" arg. Note that
// in this case the global "initial_model" that is used by default is not changed.
cassie_sim_t *cassie_sim_init(const char *modelfile, bool reinit);

// Creates an instance of the Cassie simulator with the same state as
// an existing instance.
cassie_sim_t *cassie_sim_duplicate(const cassie_sim_t *sim);

// Copies the state of one Cassie simulator to another.
void cassie_sim_copy(cassie_sim_t *dst, const cassie_sim_t *src);

// Destroys an instance of the Cassie simulator.
void cassie_sim_free(cassie_sim_t *sim);

// Deletes the saved initial model
void delete_init_model();

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

void cassie_sim_step_pd_no2khz(cassie_sim_t *c, state_out_t *y, const pd_in_t *u);

// Forward Integrate the position coordinate based on what is in qvel.
// This takes 1/2000 sec steps. Adjust the "step size" by scaling velocity
void cassie_integrate_pos(cassie_sim_t *c, state_out_t *y);

// Returns a read-write pointer to the simulator time.
double *cassie_sim_time(cassie_sim_t *sim);

double *cassie_sim_timestep(cassie_sim_t *c);

void cassie_sim_set_timestep(cassie_sim_t *c, double dt);

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

// Returns a read-write pointer to the simulator joint velocities.
// Order of the values are the same as qvel
double *cassie_sim_qacc(cassie_sim_t *c);

// Returns a read-write pointer to the simulator actuated motors.
double *cassie_sim_ctrl(cassie_sim_t *sim);

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
// cfrc[6-8]:  Contact force acting on the right foot, in world coordinates
// cfrc[9-11]: Currently zero, reserved for torque acting on the right foot
void cassie_sim_foot_forces(const cassie_sim_t *c, double cfrc[12]);

// Returns the contact forces on the left and right toes and heels.
// toe_force[0-2]:  Contact force acting on the left toe, in world coordinates
// toe_force[3-5]:  Contact force acting on the right toe, in world coordinates
// heel_force[6-8]:  Contact force acting on the left heel, in world coordinates
// heel_force[9-11]: Contact force acting on the right heel, in world coordinates
void cassie_sim_heeltoe_forces(const cassie_sim_t *c, double toe_force[6], double heel_force[6]);

// Returns whether or not Cassie is colliding with a geom in the inputted group
bool cassie_sim_geom_collision(const cassie_sim_t *c, int geom_group);

// Returns CoM velocities of the feet. Returns 12 long array, with 6 values for
// each foot (left then right) in order of 3D rotation and then 3D translation
void cassie_sim_foot_velocities(const cassie_sim_t *c, double cvel[12]);

// Returns the CoM position of the entire robot in the world frame
// (technically the subtree center of mass from the pelvis). [m]
void cassie_sim_cm_position(const cassie_sim_t *c, double cm_pos[3]);

// Returns the CoM velocity of the entire robot in the world frame
// (technically the subtree center of mass from the pelvis). [m/s]
void cassie_sim_cm_velocity(const cassie_sim_t *c, double cm_vel[3]);

// Returns 3x3 rotational intertia matrix of the robot around its center
// of mass in the pelvis frame. [kg*m^2]
void cassie_sim_centroid_inertia(const cassie_sim_t *c, double Icm[9]);

// Return the angular momentum of the robot in the world frame.
void cassie_sim_angular_momentum(const cassie_sim_t *c, double Lcm[3]);

// Return the full 32x32 mass matrix of Cassie.
void cassie_sim_full_mass_matrix(const cassie_sim_t *c, double M[1024]);

// Return the minimal actuated mass matrix of Cassie. Contains 6 for floating
// base, 5 for left leg motors, 5 for right leg motors.
void cassie_sim_minimal_mass_matrix(const cassie_sim_t *c, double M[256]);

// Return the conrod closed loop constraint jacobian and constraint violation (error) vectors
void cassie_sim_loop_constraint_info(const cassie_sim_t *c, double J_cl[192], double err_cl[6]);

// Returns CoM velocities of the inputted body specified by the input string.
// Returns 6 long array, with 6 values for each foot (left then right) in order of 3D rotation and then 3D translation
void cassie_sim_body_velocities(const cassie_sim_t *c, double cvel[6], const char* name);

// Body accelerations
void cassie_sim_body_acceleration(const cassie_sim_t *c, double accel[6], const char* name);

// Body contact force in global frame
void cassie_sim_body_contact_force(const cassie_sim_t *c, double cfrc[6], const char* name);

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

void reset_state_est(cassie_sim_t *c, state_out_t *y);

double* cassie_sim_xpos(cassie_sim_t *c, const char* name);

double* cassie_sim_xquat(cassie_sim_t *c, const char* name);

void cassie_sim_foot_orient(const cassie_sim_t *c, double corient[4]);

void cassie_sim_set_geom_name_quat(cassie_sim_t *c, const char* name, double *quat);

void cassie_sim_set_geom_name_friction(cassie_sim_t *c, const char* name, double *fric);

double *cassie_sim_get_geom_name_friction(cassie_sim_t *c, const char* name);

void cassie_sim_set_geom_name_pos(cassie_sim_t *c, const char* name, double *pos);

double *cassie_sim_geom_name_pos(cassie_sim_t *c, const char* name);

void cassie_sim_set_body_name_mass(cassie_sim_t *c, const char* name, double mass);

void cassie_sim_set_body_name_pos(cassie_sim_t *c, const char* name, double *data);

double* cassie_sim_get_body_name_pos(cassie_sim_t *c, const char* name);

double* cassie_sim_site_xpos(cassie_sim_t *c, const char* name);

void cassie_sim_site_xquat(cassie_sim_t *c, const char* name, double* xquat);

void cassie_sim_relative_pose(double pos1[3], double quat1[4],
                              double pos2[3], double quat2[4],
                              double pos2_in_pos1[3],double quat2_in_quat1[4]);

int cassie_sim_get_hfield_nrow(cassie_sim_t *c);

int cassie_sim_get_hfield_ncol(cassie_sim_t *c);

int cassie_sim_get_nhfielddata(cassie_sim_t *c);

double* cassie_sim_get_hfield_size(cassie_sim_t *c);

void cassie_sim_set_hfield_size(cassie_sim_t *c, double size[4]);

float* cassie_sim_hfielddata(cassie_sim_t *c);

void cassie_sim_set_hfielddata(cassie_sim_t *c, float* data);

void cassie_vis_set_cam(cassie_vis_t* v, const char* body_name, double zoom, double azi, double elev);

void cassie_vis_set_cam_pos(cassie_vis_t* v, double* look_point, double distance, double azi, double elev);

void cassie_sim_get_jacobian(cassie_sim_t *c, double *jac, const char* name);

void cassie_sim_get_jacobian_full(cassie_sim_t *c, double *jac, double *jac_rot, const char* name);

void cassie_sim_get_jacobian_full_site(cassie_sim_t *c, double *jac, double *jac_rot, const char* name);

void cassie_sim_just_set_const(cassie_sim_t *c);

/*******************************************************************************
 * Cassie visualizer functions
 ******************************************************************************/

// Creates an instance of the Cassie simulation visualizer. If called
// before cassie_mujoco_init, cassie_mujoco_init is called with the
// parameter NULL.
cassie_vis_t *cassie_vis_init(cassie_sim_t *sim, const char* modelfile, bool offscreen);

// Closes the visualization window without freeing the instance. After
// calling this, cassie_vis_draw can still be called, but the
// visualizer will remain closed.
void cassie_vis_close(cassie_vis_t *vis);

// Closes and frees the visualization window.
void cassie_vis_free(cassie_vis_t *vis);

// Visualizes the state of the given Cassie simulator.
bool cassie_vis_draw(cassie_vis_t *vis, cassie_sim_t *sim);

// Resize the visualization window
void cassie_vis_window_resize(cassie_vis_t *vis, int width, int height);

// Returns true if the visualizer has been closed but not freed.
bool cassie_vis_valid(cassie_vis_t *vis);

// Returns value of vis->paused
bool cassie_vis_paused(cassie_vis_t *vis);

// Returns value of vis->slowmotion
bool cassie_vis_slowmo(cassie_vis_t *vis);

// add a spherical marker for visualization purposes into scene.
void cassie_vis_add_marker(cassie_vis_t* v, double pos[3], double size[3], double rgba[4], double so3[9]);

// remove a visualization marker
void cassie_vis_remove_marker(cassie_vis_t* v, int id);

// remove all visualization markers
void cassie_vis_clear_markers(cassie_vis_t* v);

// update existing marker
void cassie_vis_update_marker_pos(cassie_vis_t* v, int id, double pos[3]);
void cassie_vis_update_marker_size(cassie_vis_t* v, int id, double size[3]);
void cassie_vis_update_marker_rgba(cassie_vis_t* v, int id, double rgba[4]);
void cassie_vis_update_marker_orient(cassie_vis_t* v, int id, double so3[9]);

// Apply inputted perturbation to any body in the vis's mjData
void cassie_vis_apply_force(cassie_vis_t *vis, double xfrc[6], const char* name);

// Does a "full reset", i.e. sets qpos to a starting position and zeros
// out all other data used for computation (like velocities, accelerations, forces)
void cassie_vis_full_reset(cassie_vis_t *sim);

// Remake the visualized mjvScene and mjrContext. Used to update cassie_vis_t
// after some change has been made to the underlying model.
void cassie_vis_remakeSceneCon(cassie_vis_t *v);

// Set height field data for vis model
void cassie_vis_set_hfielddata(cassie_vis_t *v, float* data);

float* cassie_vis_hfielddata(cassie_vis_t *c);

// Set the visualization camera to track the inputted body (specified by a string matching the name of a body defined
// in the XML model file). Also takes in as input a zoom level as well as a azimuth and elevation value controlling
// the angle of the camera.
void cassie_vis_set_cam(cassie_vis_t* v, const char* body_name, double zoom, double azi, double elev);

// initialize a video renderer
void cassie_vis_init_recording(cassie_vis_t *sim, const char* videofile, int width, int height);

//Record Current frame
void cassie_vis_record_frame(cassie_vis_t *sim);

// close a video renderer
void cassie_vis_close_recording(cassie_vis_t *sim);

// Depth Functions
void cassie_vis_init_depth(cassie_vis_t *v, int width, int height);

void cassie_vis_init_rgb(cassie_vis_t *v, int width, int height);

float* cassie_vis_draw_depth(cassie_vis_t *v, cassie_sim_t *c, int width, int height);

unsigned char* cassie_vis_get_rgb(cassie_vis_t *v, cassie_sim_t *c, int width, int height);


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
