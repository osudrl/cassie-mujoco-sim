#!/usr/bin/env python3

from cassiemujoco import *
import time

# Initialize cassie simulation
sim = CassieSim()
vis = CassieVis()

# Set control parameters
u = pd_in_t()
u.leftLeg.motorPd.torque[3] = 0 # Feedforward torque
u.leftLeg.motorPd.pTarget[3] = -2
u.leftLeg.motorPd.pGain[3] = 1000
u.leftLeg.motorPd.dTarget[3] = -2
u.leftLeg.motorPd.dGain[3] = 100
u.rightLeg.motorPd = u.leftLeg.motorPd

# Hold pelvis in place
sim.hold()

# Record time
t = time.monotonic()

# Run until window is closed
while True:
    for _ in range(33):
        y = sim.step_pd(u)

    if not vis.draw(sim):
        break

    while time.monotonic() - t < 1/60:
        time.sleep(0.001)
    t = time.monotonic()
