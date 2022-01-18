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

"""
Test the end-effector Jacobian, don't care
1. dynamically consistent (Mx*J*Mx^-1);
2. closed-loop constraints;
3. rotational components. 
Only to get the rough estimate of the Jacobian matrix on motors.
"""

# Initialize cassie simulation
sim = CassieSim("../model/cassie.xml")
vis = CassieVis(sim)
# Set control parameters
u = pd_in_t()
test_jac_control = True
test_random = True 

if test_jac_control:
    qpos = sim.qpos()
    qpos[2] = 1.5
    sim.set_qpos(qpos)
    sim.hold()

# Record time
t = time.monotonic()
count = 0
ltarget = np.array([0,  0.13, -0.8])
rtarget = np.array([0, -0.13, -0.5])
kp = np.array([70,  70,  100,  100,  50])
kd = np.array([7.0, 7.0, 8.0,  8.0, 5.0])

# Run until window is closed or vis is quit
draw_state = vis.draw(sim)
vel_idx      = [6, 7, 8, 12, 18, 19, 20, 21, 25, 31]
pos_idx      = [7, 8, 9, 14, 20, 21, 22, 23, 28, 34]

ts_noise_up = np.array([[0,    0, 0.13, 0.13, 0],
                        [0.13, 0, 0.00, 0.00, 0],
                        [0,    0, 0.13, 0.13, 0]])

ts_noise = np.block([
    [ts_noise_up,     np.zeros((3,5))],
    [np.zeros((3,5)), ts_noise_up]
    ])

offset = np.array([0.0045, 0.0, 0.4973, -1.1997, -1.5968, 0.0045, 0.0, 0.4973, -1.1997, -1.5968])

while draw_state:
    if not vis.ispaused():
        for i in range(60):
            jacpl = sim.get_jacobian(name='left-foot').reshape(3, -1)
            jacpr = sim.get_jacobian(name='right-foot').reshape(3, -1)
            jacp = np.concatenate((jacpl, jacpr))
            jacp_motor = jacp.take(vel_idx, axis=1)
            jdag = np.linalg.pinv(jacp_motor)

            if test_jac_control:
                lpos = np.array(sim.foot_pos()[0:3]) - np.array(sim.qpos()[0:3])
                rpos = np.array(sim.foot_pos()[3:6]) - np.array(sim.qpos()[0:3])
                dxl = ltarget - lpos
                dxr = rtarget - rpos
                # print(np.dot(jdag[:,0:3], dxl).shape)
                # print(np.dot(jdag[:,3:6], dxr).shape)
                dq = np.dot(jdag[:,0:3], dxl) + np.dot(jdag[:,3:6], dxr)
                # print(dq.shape)
                # print(dq)
                # print(lpos)
                qpos = sim.qpos()
                mpos = [qpos[i] for i in pos_idx]

                for i in range(5):
                    u.leftLeg.motorPd.pGain[i]  = kp[i] * 0.1 
                    u.rightLeg.motorPd.pGain[i] = kp[i] * 0.1
                    u.leftLeg.motorPd.dGain[i]  = kd[i] * 0.1
                    u.rightLeg.motorPd.dGain[i] = kd[i] * 0.1
                    u.leftLeg.motorPd.torque[i]  = 0  # Feedforward torque
                    u.rightLeg.motorPd.torque[i] = 0
                    u.leftLeg.motorPd.pTarget[i]  = dq[i] + mpos[i]
                    u.rightLeg.motorPd.pTarget[i] = dq[i+5] + mpos[i+5]
                    u.leftLeg.motorPd.dTarget[i]  = 0
                    u.rightLeg.motorPd.dTarget[i] = 0
                y = sim.step_pd(u)
            else:
                action = np.random.uniform(-10, 10, size=10)
                for i in range(5):
                    u.leftLeg.motorPd.pGain[i]  = kp[i]
                    u.rightLeg.motorPd.pGain[i] = kp[i]
                    u.leftLeg.motorPd.dGain[i]  = kd[i]
                    u.rightLeg.motorPd.dGain[i] = kd[i]
                    u.leftLeg.motorPd.torque[i]  = 0  # Feedforward torque
                    u.rightLeg.motorPd.torque[i] = 0
                    u.leftLeg.motorPd.pTarget[i]  = action[i] + offset[i]
                    u.rightLeg.motorPd.pTarget[i] = action[i+5] + offset[i+5]
                    u.leftLeg.motorPd.dTarget[i]  = 0
                    u.rightLeg.motorPd.dTarget[i] = 0
                y = sim.step_pd(u)

        sd_final = np.matmul(jdag, ts_noise)
        # print(sd_final.shape)
        # print("new js noise matrix")
        # for i in range(10):
        #     for j in range(10):
        #         print("{: 3.2f}".format(sd_final[i][j]), end=" ")
        #     print("\n")
        # input()

    draw_state = vis.draw(sim)
    count += 1

    while time.monotonic() - t < 60*0.0005:
        time.sleep(0.0001)
    t = time.monotonic()

