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
sim = CassieSim("../model/cassie.xml")
vis = CassieVis(sim, "../model/cassie.xml")

cassie_reload_xml(str.encode("../model/cassie_noise_terrain.xml"))
sim2 = CassieSim("../model/cassie_noise_terrain.xml")
vis2 = CassieVis(sim2, "../model/cassie_noise_terrain.xml")
draw_state2 = vis2.draw(sim2)

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
sim.set_geom_friction([.1, 1e-3, 1e-4], "floor")

# Run until window is closed or vis is quit
draw_state = vis.draw(sim)
feet_vel = np.zeros(12)
while draw_state and draw_state2:
    if not vis.ispaused():
        # if 50 < count < 80:
        #     vis.apply_force([0, 0, 500, 0, 0, 0], "cassie-pelvis")
        #     print("applying force", count)
        # else:
        #     vis.apply_force([0, 0, 0, 0, 0, 0], "cassie-pelvis")
        for i in range(60):
            y = sim.step_pd(u)
        # print("left foot quat: ", sim.xquat("left-foot"))
        sim.foot_vel(feet_vel)
        print("left foot vel: ", feet_vel[3:6])
        print("right foot vel: ", feet_vel[9:12])
        count += 1

    draw_state = vis.draw(sim)
    draw_state2 = vis2.draw(sim2)

    # while time.monotonic() - t < 1/60:
    time.sleep(1/30)
    # t = time.monotonic()
