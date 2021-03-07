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

# Initialize cassie simulation
sim = CassieSim("../model/cassie_mass.xml")
sim.set_body_mass(10, name="load_mass")
print(sim.nq)
# sim2 = CassieSim("../model/cassie.xml")
# print(ctypes.addressof(sim.c))
# print(ctypes.addressof(sim2.c))

# joint_filter = cassie_sim_joint_filter(sim.c)
# joint_filter_s = sim.get_joint_filter()
# joint_filter2 = sim2.get_joint_filter()
# joint_filter_s[0].x[0] = 1
# print(joint_filter2[0].x[0])
# exit()
# sim.qpos()
# sim2.qpos()
# exit()
# print(joint_filter_s)
# print(joint_filter2)
# print(joint_filter_s[0].x[0:4])

# in_filters = [joint_filter_t()] * 6
# in_filters = (joint_filter_t*6)()
# # print(in_filters)
# for i in range(4):
#     in_filters[0].x[i] = 1
# # in_x = np.ones(4)
# # in_filter.x = in_x.astype(ctypes.c_double)
# # print(in_filters)
# # ctypes.cast(in_filters, ctypes.POINTER(joint_filter_t))
# # cassie_sim_set_joint_filter(sim.c, in_filters)
# sim.set_joint_filter(in_filters)
# # joint_filter = cassie_sim_joint_filter(sim.c)
# joint_filter = sim.get_joint_filter()
# joint_filter2 = sim2.get_joint_filter()
# print("set filter")
# print(joint_filter[0].x[0:5])
# print(joint_filter[1].x[0:5])
# print(joint_filter[2].x[0:5])
# # exit()
# print("orig", joint_filter_s[0].x[0:4])
# # exit()

# sim.set_joint_filter(joint_filter_s)
# joint_filter = sim.get_joint_filter()
# print("set filter")
# print(joint_filter[0].x[0:5])
# print(joint_filter[1].x[0:5])

# exit()

# d_filters = (drive_filter_t*10)()
# for i in range(9):
#     d_filters[0].x[i] = 1
# # in_x = np.ones(4)
# # in_filter.x = in_x.astype(ctypes.c_double)
# # ctypes.cast(in_filters, ctypes.POINTER(joint_filter_t))
# cassie_sim_set_drive_filter(sim.c, d_filters)
# drive_filter = cassie_sim_drive_filter(sim.c)
# print("set filter")
# print(drive_filter[0].x[0:9])
# print(drive_filter[1].x[0:9])

# t_delay = (ctypes.c_double * 60)()
# cassie_sim_torque_delay(sim.c, t_delay)
# set_t = np.zeros((10, 6))
# set_t[0, :] = np.ones(6)
# print("set_t", set_t)
# set_t_arr = (ctypes.c_double * 60)(*set_t.flatten())
# print(t_delay[0:60])
# cassie_sim_set_torque_delay(sim.c, ctypes.cast(set_t_arr, ctypes.POINTER(ctypes.c_double)))
# cassie_sim_torque_delay(sim.c, t_delay)
# # test = np.zeros((10, 6))
# print(t_delay[0:60])
# test = np.array(t_delay[:]).reshape((10, 6))
# print(test)

# print(in_filter.x)

# exit()

# data = np.ones((10, 10))
# size = 250
# mid = int(size/2)
# data = np.random.rand(size, size)
# data[mid-5:mid+5, mid-5:mid+5] = 0
# # print(data)
# sim.set_hfield_data(data.flatten())
# hfield_data = sim.get_hfield_data()
# sim = CassieSim("../model/cassie_hfield.xml")


# print(sim.get_hfield_ncol())
# print(sim.get_hfield_nrow())
# print(sim.get_nhfielddata())
# print(sim.get_hfield_size())
# data = np.ones((10, 10))
# # data = np.random.rand(10, 10)
# data[:, 0] = 0
# print(data)
# sim.set_hfield_data(data.flatten())
# hfield_data = sim.get_hfield_data()
# print(hfield_data)
vis = CassieVis(sim)


# sim.set_hfield_size([100, 100, .15, .001])
# print(sim.get_hfield_size())

# cassie_reload_xml(str.encode("../model/cassie_noise_terrain.xml"))
# sim2 = CassieSim("../model/cassie_noise_terrain.xml", reinit=True)
# vis2 = CassieVis(sim2, "../model/cassie_noise_terrain.xml")
# draw_state2 = vis2.draw(sim2)


# Set control parameters
u = pd_in_t()
# u.leftLeg.motorPd.torque[3] = 0 # Feedforward torque
# u.leftLeg.motorPd.pTarget[3] = -2
# u.leftLeg.motorPd.pGain[3] = 100
# u.leftLeg.motorPd.dTarget[3] = -2
# u.leftLeg.motorPd.dGain[3] = 10
# u.rightLeg.motorPd = u.leftLeg.motorPd

# Hold pelvis in place
# sim.hold()

# Record time
t = time.monotonic()
count = 0

# quat_set = euler2quat(z=0, y=np.pi/180*5, x=np.pi/180*5)
# sim.set_geom_quat(quat_set, "floor")
# sim.set_geom_friction([.1, 1e-3, 1e-4], "floor")

# Run until window is closed or vis is quit
draw_state = vis.draw(sim)
feet_vel = np.zeros(12)
rfoot_quat = np.zeros(4)
rfoot_body_quat = np.zeros(4)
pel_vel = np.zeros(6)
# sim.hold()
# sim.set_body_mass(5, name="right-foot")
# vis.set_cam("cassie-pelvis", 3, 90, -20)
# (90,0) size view
while draw_state:# and draw_state2:
    if not vis.ispaused():
        # if 50 < count < 80:
        #     vis.apply_force([0, 0, 500, 0, 0, 0], "cassie-pelvis")
        #     print("applying force", count)
        # else:
        #     vis.apply_force([0, 0, 0, 0, 0, 0], "cassie-pelvis")
        for i in range(60):
            y = sim.step_pd(u)
        # sim.hold()
        qpos = sim.qpos_full()
        qvel = sim.qvel_full()
        print("mass: ", qpos[2])
        # print("pel z:", qpos[2])
        # print("left foot quat: ", sim.xquat("left-foot"))
        # qvel = sim.qvel()
        # sim.foot_vel(feet_vel)
        # sim.foot_quat(rfoot_quat)
        # rfoot_body_quat = sim.xquat("right-foot")
        # sim.body_vel(pel_vel, 'cassie-pelvis')
        # print("r foot quat: ", rfoot_quat)
        # print("rfoot_body_quat: ", rfoot_body_quat)
        # print("state est quat: ", y.rightFoot.orientation[:])
        # print("left foot vel: ", feet_vel[3:6])
        # print("right foot vel: ", feet_vel[9:12])
        # print("state est vel: ", y.rightFoot.footTranslationalVelocity[:])
        # print("diff: ", np.linalg.norm(feet_vel[9:12]-y.rightFoot.footTranslationalVelocity[:]))
        # print("pel com vel: ", pel_vel[3:])
        # print("pel qvel: ", qvel[0:3])
        # print("state est pel vel: ", y.pelvis.translationalVelocity[:])
        # print("diff: ", np.linalg.norm(np.array(y.pelvis.translationalVelocity[:])-qvel[0:3]))
        # count += 1

    draw_state = vis.draw(sim)
    # draw_state2 = vis2.draw(sim2)

    # while time.monotonic() - t < 1/60:
    time.sleep(1/30)
    # t = time.monotonic()
