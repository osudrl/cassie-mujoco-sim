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

# Null controller
def controller(y):
    return pd_in_t()

# Set up UDP connection
cassie = CassieUdp(remote_addr='127.0.0.1')

received_data = False

# Listen/respond loop
while True:
    if not received_data:
        # Send packets until the simulator responds
        print('Connecting...')
        y = None
        while y is None:
            cassie.send_pd(pd_in_t())
            time.sleep(0.001)
            y = cassie.recv_newest_pd()
        received_data = True
        print('Connected!\n')
    else:
        # Wait for new data
        y = cassie.recv_wait_pd()

    # Print connection stats
    print('\033[F\033[Jdelay: {}, diff: {}'.format(cassie.delay(),
                                                   cassie.seq_num_in_diff()))

    # Run controller
    u = controller(y)

    # Send response
    cassie.send_pd(u)
