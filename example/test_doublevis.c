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

#include "cassiemujoco.h"
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

/* DEVEL NOTES
Note that when visualizing more than one window using the same sim model, interactive applied perturbations will be visualized
different in the other window. This is due to the fact that each window (cassie_vis object) has its own mjvPerturb object,
which is used to do the "correct" perturbation visualization (some arrow will still be shown in the other window automatically, 
but the scaling and color is different). Thus, second cassie_vis object doesn't get/update its own mjvPerturb when the first 
cassie_vis is being perturbed. Not sure how to fix this. Even if we make mjvPerturb part of cassie_sim instead of cassie_vis, 
mjvPerturb is changed in the GLFW callbacks, so then would have to have access to the cassie_sim object in the callbacks. So 
would have the have the cassie_vis object have the pointer to the whole cassie_sim object instead of mjData and mjModel. Seems 
bulky though. Can't make mjvPerturb a global var either, since it would break having two separate cassie_vis with their own
cassie_sim each. Regardless, simulation is unaffected, the difference is purely visual.
*/

static long get_microseconds(void) {
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return now.tv_sec * 1000000 + now.tv_nsec / 1000;
}

int main(void)
{
    const char modelfile[] = "../model/cassie.xml";
    cassie_sim_t *c = cassie_sim_init(modelfile, false);
    cassie_vis_t *v = cassie_vis_init(c, modelfile, false);
    cassie_vis_t *v2 = cassie_vis_init(c, modelfile, false);

    state_out_t y;
    pd_in_t u = {0};
    
    bool draw_state = cassie_vis_draw(v, c);
    bool draw_state2 = cassie_vis_draw(v2, c);
    long start_t = get_microseconds();
    long sleep_time = 0;
    while (draw_state || draw_state2) {
        start_t = get_microseconds();
        if (!cassie_vis_paused(v) & !cassie_vis_paused(v2)) {
            // Base 40 fps
            for (int i = 0; i < 50; i++) {
                cassie_sim_step_pd(c, &y, &u);
            }
        }
        // Only draw if the window is open
        if (draw_state) {
            draw_state = cassie_vis_draw(v, c);
        }
        if (draw_state2) {
            draw_state2 = cassie_vis_draw(v2, c);
        }
        sleep_time = 25000 - (get_microseconds() - start_t);
        if (sleep_time < 0){
            sleep_time = 0;
        }
        usleep(sleep_time);
    }

    // Should free vis before freeing sim. 
    cassie_vis_free(v);
    cassie_vis_free(v2);
    cassie_sim_free(c);
    cassie_cleanup();

    return 0;
}
