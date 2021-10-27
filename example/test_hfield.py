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

# Initialize cassie simulation
sim = CassieSim("../model/cassie_hfield.xml")
vis = CassieVis(sim)

# Set control parameters
u = pd_in_t()

# Record time
t = time.monotonic()
count = 0

# Generate random terrain. All hfield data must be scaled between 0-1, and max height from hfield_size will set the scaling.
# Indexing starts at negative x negative y (back right corner of the hfield in relation to where Cassie faces foward.)
# needs to be flattened before being passed to set_hfield_data
nrows = sim.get_hfield_nrow()
ncols = sim.get_hfield_ncol()
rand_hdata = np.random.random((nrows, ncols))
# Set middle of hfield (where Cassie starts) to be flat
rand_hdata[nrows//2-5:nrows//2 + 5, ncols//2-5:ncols//2+5] = 0
sim.set_hfield_data(rand_hdata.flatten(), vis.v)

# Run until window is closed or vis is quit
draw_state = vis.draw(sim)

while draw_state:# and draw_state2:
    if not vis.ispaused():
        for i in range(60):
            y = sim.step_pd(u)

    draw_state = vis.draw(sim)

    while time.monotonic() - t < 60*0.0005:
        time.sleep(0.0001)
    t = time.monotonic()

