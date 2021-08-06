#!/usr/bin/env python3

# Copyright (c) 2018 Dynamic Robotics Laboratory
#
# Permission to use, copy, modify, and distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

from cassiemujoco import *
from cassiemujoco_ctypes import joint_filter_t, drive_filter_t
import time
import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib import cm
from collections import namedtuple
import json
from cassieXboxController import cassieXboxController
from optimalCoords.optimal_cassie import load_data, interpolate_so3
from scipy.spatial.transform import Rotation as R

MOTOR_POS_IDX = [7,8,9,14,20,21,22,23,28,34]
MOTOR_VEL_IDX = [7,8,9,14,20,21,22,23,28,34]
PASSIVE_VEL_IDX = [9,10,11,14,22,23,24,27]

DynamicInfo = namedtuple("DynamicInfo", "qpos motorPos M_min I_centroid cm_pos")


def euler2quat(z=0, y=0, x=0):

    z = z/2.0
    y = y/2.0
    x = x/2.0
    cz = math.cos(z)
    sz = math.sin(z)
    cy = math.cos(y)
    sy = math.sin(y)
    cx = math.cos(x)
    sx = math.sin(x)
    result =  np.array([
             cx*cy*cz - sx*sy*sz,
             cx*sy*sz + cy*cz*sx,
             cx*cz*sy - sx*cy*sz,
             cx*cy*sz + sx*cz*sy])
    if result[0] < 0:
    	result = -result
    return result

def forwardUpdateClosedLoop(sim, vis, motorPos):
    u = pd_in_t()
    # sim.full_reset()
    qpos = sim.qpos()
    qvel = sim.qvel()
    qpos[0] = 0
    qpos[1] = 0
    qpos[2] = 1.5
    qpos[3] = 1
    qpos[4] = 0
    qpos[5] = 0
    qpos[6] = 0

    for i in range(5):
        qpos[MOTOR_POS_IDX[i]] = motorPos[i]
        qpos[MOTOR_POS_IDX[i+5]] = motorPos[i+5]     
    
    sim.set_qpos(qpos)
    sim.set_qvel(np.zeros((len(qvel), 1)))
    sim.hold()
    nStep = 0
    err_c = 10
    while nStep < 500 and np.linalg.norm(err_c) > 1e-5:
        J_c = sim.constraint_jacobian()
        err_c = sim.constraint_error()
        J_passive_c = np.zeros(J_c.shape)
        J_passive_c[:, PASSIVE_VEL_IDX] = J_c[:, PASSIVE_VEL_IDX]

        q_vel = np.linalg.lstsq(J_passive_c, -200*err_c, rcond=None)

        sim.set_qvel(q_vel[0])
        sim.integrate_pos()

        nStep = nStep + 1  
        # draw_state = vis.draw(sim)

    qpos_final = sim.qpos()
    # print("Finished in " + str(nStep) + " steps with error norm of " + str(np.linalg.norm(err_c)))
    return qpos_final


def getAllDynamicInfo(sim, vis, motor_pos):
    qpos_result = forwardUpdateClosedLoop(sim, vis, motor_pos)
    sim.set_qpos(qpos_result)
    M_minimal = sim.minimal_mass_matrix()
    centroid_interia = sim.centroid_inertia()
    centerofMass_pos = sim.center_of_mass_position()

    dynInf = {}
    dynInf['qpos']=qpos_result, 
    dynInf['motorPos']=motor_pos, 
    dynInf['M_min']= M_minimal.tolist()
    dynInf['I_centroid']=centroid_interia
    dynInf['cm_pos']=centerofMass_pos

    return dynInf

def writeDynInfoJSON(dynInfo_list):
    # print(json.dumps({'q_pos':dynInfo_list[0].qpos, 'motorPos':dynInfo_list[0].motorPos,
    #                   'M_minimal':dynInfo_list[0].M_min, 'I_centroid':dynInfo_list[0].I_centroid,
    #                   'cm_pos':dynInfo_list[0].cm_pos}))
    with open('cassieInertia.txt', 'w') as outfile:
        json.dump(dynInfo_list, outfile)

def update_motor_pos(controller, motor_pos):
    gain = 0.01
    controller.process_events()
    flip, vals = controller.get_controller_state()
    if flip == 1:
        motor_pos[0] += vals[1]*gain
        motor_pos[2] += vals[2]*gain
        motor_pos[3] += vals[0]*gain
    else:
        motor_pos[5] += vals[1]*gain
        motor_pos[7] += vals[2]*gain
        motor_pos[8] += vals[0]*gain
    print(flip)
    return motor_pos



trans_dict = load_data()
alpha = np.array([0.1, 0.75, -1.2, 0, 0.75, -1.2])
controller = cassieXboxController()


# Initialize cassie simulation
sim = CassieSim("../model/cassie_no_grav.xml")
print(sim.nq)
vis = CassieVis(sim)

# Set control parameters
u = pd_in_t()

# Record time
t = time.monotonic()
count = 0

# Run until window is closed or vis is quit
draw_state = vis.draw(sim)

sim.full_reset()
qpos = sim.qpos()


motor_pos = []
for i in range(10):
    motor_pos.append(qpos[MOTOR_POS_IDX[i]])


vis.start_video_recording("pyVideoTest", 1280, 720)
i = 0
while controller.button_state['M'] == 0:

    motor_pos = update_motor_pos(controller, motor_pos)
    alpha[0] = motor_pos[0]
    alpha[1] = motor_pos[2]
    alpha[2] = motor_pos[3]
    alpha[3] = motor_pos[5]
    alpha[4] = motor_pos[7]
    alpha[5] = motor_pos[8]

    qpos_result = forwardUpdateClosedLoop(sim, vis, motor_pos)

    interp = interpolate_so3(trans_dict, alpha)
    r = R.from_matrix(interp['rot_mat'])
    body_orientation_offset = r.as_quat()  
    qpos_result[3] = body_orientation_offset[0]
    qpos_result[4] = body_orientation_offset[1]
    qpos_result[5] = body_orientation_offset[2]
    qpos_result[6] = body_orientation_offset[3]

    sim.set_qpos(qpos_result)
    draw_state = vis.draw(sim)

    i = i+1
    if i%5 == 0:
        vis.record_frame()
    time.sleep(1.0/(60*5))


vis.close_video_recording()
    


# hip_list = np.radians(np.linspace(-20, 60, num=N_grid))
# knee_list = np.radians(np.linspace(-100, -60, num=N_grid))

# error = np.zeros((N_grid,N_grid))

# dynInfo_list = []
# for left_hip_idx in range(N_grid):
#     for left_knee_idx in  range(N_grid):

#         left_hip_pitch_angle = hip_list[left_hip_idx]
#         left_knee_angle = knee_list[left_knee_idx]

#         motor_pos[2] = left_hip_pitch_angle
#         motor_pos[3] = left_knee_angle
#         motor_pos[7] = left_hip_pitch_angle
#         motor_pos[8] = left_knee_angle
        
#         dynInfo = getAllDynamicInfo(sim, vis, motor_pos)
#         dynInfo_list.append(dynInfo)



# writeDynInfoJSON(dynInfo_list)

# Plot the surface.

# fig, axs = plt.subplots(nrows=1, ncols=1)
# temp_imshow = axs.imshow(error, extent = (hip_list[0], hip_list[-1], knee_list[0], knee_list[-1]))
# fig.colorbar(temp_imshow, ax=axs)
# plt.show()


