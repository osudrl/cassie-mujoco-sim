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

static long get_microseconds(void) {
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return now.tv_sec * 1000000 + now.tv_nsec / 1000;
}

int main(void)
{
    const char* modelfile = "../model/cassie.xml";
    cassie_sim_t *c = cassie_sim_init(modelfile, false);
    cassie_vis_t *v = cassie_vis_init(c, modelfile, false);
    // cassie_sim_hold(c);

    state_out_t y;
    pd_in_t u = {0};

    long start_t = get_microseconds();
    long sleep_time = 0;
    bool draw_state = cassie_vis_draw(v, c);
    int count = 0;
    double pos[3] = {1.0, 0.0, 1.0};
    double size[3] = {0.2, 0.2, 0.2};
    double rgba[4] = {0.8, 0.1, 0.1, 1.0};
    double so3[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    char* geom_type = "sphere";
    char* name = "foo";
    int marker_id = cassie_vis_add_marker(v, geom_type, name, pos, size, rgba, so3);
    pos[2] = 0.5;
    int marker_id2 = cassie_vis_add_marker(v, geom_type, name, pos, size, rgba, so3);
    printf("marker id %i\n", marker_id2);
    while (draw_state) {
        start_t = get_microseconds();
        if (!cassie_vis_paused(v)) {
            // if (count == 20) {
            //     count = 0;

            //     for (int i=0; i<100; i++) {
            //         cassie_sim_set_const(c);
            //         reset_state_est(c, &y);
            //     }
            //     printf("Reset\n");
            // }
            // double* qvel = cassie_sim_qvel(c);
            // printf("pel vel: %f %f %f\n", y.pelvis.translationalVelocity[0], y.pelvis.translationalVelocity[1], y.pelvis.translationalVelocity[2]);
            // printf("true vel: %f %f %f\n", qvel[0], qvel[1], qvel[2]);
            for (int i = 0; i < 50; i++) {
                cassie_sim_step_pd(c, &y, &u);
            }
            // for (int i = 0; i<3; i++

            count += 1;
            if (count == 30) {
                cassie_vis_remove_marker(v, 0);
                pos[0] = 0.3;
                pos[1] = 0.2;
                pos[2] = 0.3;
                cassie_vis_update_marker_pos(v, 1, pos);
                cassie_vis_update_marker_type(v, 1, "box");
                cassie_vis_update_marker_name(v, 1, "new marker");
            }
        }
        draw_state = cassie_vis_draw(v, c);
        sleep_time = 25000 - (get_microseconds() - start_t);
        if (sleep_time < 0){
            sleep_time = 0;
        }
        usleep(sleep_time);
    }

    cassie_sim_free(c);
    cassie_vis_free(v);
    cassie_cleanup();

    return 0;
}
