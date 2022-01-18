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
    qpos[2] = 0.5
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
    print("Finished in " + str(nStep) + " steps with error norm of " + str(np.linalg.norm(err_c)))
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
motor_pos_nominal = []
for i in range(10):
    motor_pos_nominal.append(qpos[MOTOR_POS_IDX[i]])



N_grid_hip_roll = 3
N_grid_knee = 3
N_grid_hip = 3
hip_list = np.linspace(-0.27, 1.25, num=N_grid_hip)
knee_list = np.linspace(-1.90, -0.9, num=N_grid_knee)
hip_roll_left_list = np.linspace(-0.18, 0.30, num=N_grid_hip_roll)
hip_roll_right_list = np.linspace(-0.30, 0.18, num=N_grid_hip_roll)


# for idx in range(N_grid):
#     motor_pos[0] = hip_roll[idx]
#     motor_pos[5] = hip_roll[idx]
#     sim.full_reset()

#     dynInfo = getAllDynamicInfo(sim, vis, motor_pos)
#     print('Hip Motor: ', hip_roll[idx])
#     input('')


header_info = {}
header_info['grid_dimensions'] = [N_grid_hip_roll, N_grid_hip, N_grid_knee,N_grid_hip_roll, N_grid_hip, N_grid_knee]
header_info['right_hip_roll_vals'] = hip_roll_right_list.tolist()
header_info['right_hip_pitch_vals'] = hip_list.tolist()
header_info['right_knee_vals'] = knee_list.tolist()
header_info['left_hip_roll_vals'] = hip_roll_left_list.tolist()
header_info['left_hip_pitch_vals'] = hip_list.tolist()
header_info['left_knee_vals'] = knee_list.tolist()


# writeDynInfoJSON(header_info)

dynInfo_list = []
for right_hip_roll_idx in range(N_grid_hip_roll):
    for right_hip__pitch_idx in range(N_grid_hip):
        for right_knee_idx in range(N_grid_knee):
            for left_hip_roll_idx in range(N_grid_hip_roll):
                for left_hip__pitch_idx in range(N_grid_hip):
                    for left_knee_idx in range(N_grid_knee):
                        
                        motor_pos = motor_pos_nominal
                        motor_pos[0] = hip_roll_left_list[left_hip_roll_idx]
                        motor_pos[2] = hip_list[left_hip__pitch_idx]
                        motor_pos[3] = knee_list[left_knee_idx]
                        motor_pos[5] = hip_roll_right_list[right_hip_roll_idx]
                        motor_pos[7] = hip_list[right_hip__pitch_idx]
                        motor_pos[8] = knee_list[right_knee_idx]

                        sim.full_reset()
                        idx_key = [right_hip_roll_idx, right_hip__pitch_idx, right_knee_idx, left_hip_roll_idx, left_hip__pitch_idx, left_knee_idx]
                        dynInfo = getAllDynamicInfo(sim, vis, motor_pos)
                        dynInfo['grid_indices'] = idx_key
                        dynInfo_list.append(dynInfo)

writeDynInfoJSON((header_info, dynInfo_list))

# Plot the surface.

# fig, axs = plt.subplots(nrows=1, ncols=1)
# temp_imshow = axs.imshow(error, extent = (hip_list[0], hip_list[-1], knee_list[0], knee_list[-1]))
# fig.colorbar(temp_imshow, ax=axs)
# plt.show()


