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
sim = CassieSim("../model/cassie_depth.xml")
vis = CassieVis(sim)
visd = CassieVis(sim, offscreen=True)
width = 300
height = 300
visd.window_resize(width, height)
visd.attach_cam(cam_name='egocentric')
visd.init_depth(width, height)

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


# Run until window is closed or vis is quit
draw_state = vis.draw(sim)
feet_vel = np.zeros(12)
rfoot_quat = np.zeros(4)
rfoot_body_quat = np.zeros(4)
pel_vel = np.zeros(6)
count = 0

while draw_state:# and draw_state2:
    if not vis.ispaused():
        for i in range(60):
            y = sim.step_pd(u)
        # sim.hold()
        # qpos = np.array(sim.qpos_full())
        # qvel = np.array(sim.qvel_full())
        # print("mass: ", qpos[0:2] - qpos[35:37])
        # print(qpos[35:38])
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
    visd.draw(sim)
    depth_ptr = visd.draw_depth(sim, width=width, height=height)
    depth = np.ctypeslib.as_array(depth_ptr, shape=(width*height,)).reshape((1,1,width,height))
    # print(min(depth), max(depth))
    # plt.imshow(np.flip(depth[0,0,:,:],0), cmap='hot', interpolation='nearest')
    # plt.show()
    # plt.savefig('./figs/img_{}.png'.format(count))
    count += 1

    while time.monotonic() - t < 60*0.0005:
        time.sleep(0.0001)
    t = time.monotonic()

