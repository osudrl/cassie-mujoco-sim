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
#include <unistd.h>
#include <math.h>

static long get_microseconds(void) {
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return now.tv_sec * 1000000 + now.tv_nsec / 1000;
}

void shift_terrain_x(float* hdata, int offset, int nrow, int ncol) {
    if (offset == 1) {  // Shift forward
        for (int j = 1; j < ncol; j++) {
            for (int i = 0; i < nrow; i++) {
                hdata[(ncol-j)+ncol*i] = hdata[(ncol-j-1)+ncol*i];
            }
        }
    } else if (offset == -1) {  // Shift backwards
        for (int j = 0; j < ncol-1; j++) {
            for (int i = 0; i < nrow; i++) {
                hdata[(j)+ncol*i] = hdata[(j+1)+ncol*i];
            }
        }
    } else {
        printf("wrong offset\n");
    }
}

void shift_terrain_y(float* hdata, int offset, int nrow, int ncol) {
    if (offset == 1) {  // Shift left
        for (int j = 1; j < nrow; j++) {
            for (int i = 0; i < ncol; i++) {
                hdata[ncol*(nrow-j)+i] = hdata[ncol*(nrow-j-1)+i];
            }
        }
    } else if (offset == -1) {  // Shift right
        for (int j = 0; j < nrow-1; j++) {
            for (int i = 0; i < ncol; i++) {
                hdata[ncol*j+i] = hdata[ncol*(j+1)+i];
            }
        }
    } else {
        printf("wrong offset\n");
    }
}

int main(void)
{
    const char modelfile[] = "../model/cassie_hfield.xml";
    cassie_sim_t *c = cassie_sim_init(modelfile, false);
    cassie_vis_t *v = cassie_vis_init(c, modelfile, false);
    time_t t;
    srand((unsigned) time(&t));
    int nrow = cassie_sim_get_hfield_nrow(c);
    int ncol = cassie_sim_get_hfield_ncol(c);
    double* hfield_size = cassie_sim_get_hfield_size(c);
    double x_size = hfield_size[0];
    double y_size = hfield_size[1];
    double x_incr = x_size / ncol;  // Due to mujoco data convention (column major) divide by ncol, not nrow
    double y_incr = y_size / nrow;
    double* hfield_pos = cassie_sim_get_body_name_pos(c, "floor");
    int hdata_size = nrow*ncol;
    float hdata[hdata_size];
    for (int i = 0; i < hdata_size; i++) {
        // hdata[i] = 0;
        hdata[i] = (float) rand() / RAND_MAX;
    }
    // Y left
    // for (int i = 0; i < ncol; i++) {
    //     hdata[ncol*(nrow-10)+i] = 1;
    // }
    // X behind
    // for (int i = 0; i < nrow; i++) {
    //     hdata[(ncol-6)+ncol*i] = 1;
    // }
    // X In front
    // for (int i = 0; i < nrow; i++) {
    //     hdata[ncol*i] = 1;
    // }
    // shift_terrain_x(hdata, -1, nrow, ncol);
    // Y right
    // for (int i = 0; i < ncol; i++) {
    //     hdata[i] = 1;
    // }
    // shift_terrain_y(hdata, -1, nrow, ncol);
    // hdata[ncol-1] = 1;
    cassie_sim_set_hfielddata(c, hdata);
    cassie_vis_remakeSceneCon(v);

    state_out_t y;
    pd_in_t u = {0};

    bool draw_state = cassie_vis_draw(v, c);
    long start_t = get_microseconds();
    long sleep_time = 0;
    double* qpos;
    double h_pos[3] = {hfield_pos[0], hfield_pos[1], -0.1};
    while (draw_state) {
        start_t = get_microseconds();
        if (!cassie_vis_paused(v)) {   
            for (int i = 0; i < 50; i++) {
                cassie_sim_step_pd(c, &y, &u);
            }
            qpos = cassie_sim_qpos(c);
            double x_diff = qpos[0] - hfield_pos[0];
            double y_diff = qpos[1] - hfield_pos[1];
            if (x_diff > x_incr) {
                shift_terrain_x(hdata, -1, nrow, ncol);
                for (int i = 0; i < nrow; i++) {
                    hdata[(ncol-1)+ncol*i] = (float) rand() / RAND_MAX;
                }
                h_pos[0] += x_incr;
            } else if (x_diff < -x_incr) {
                shift_terrain_x(hdata, 1, nrow, ncol);
                for (int i = 0; i < nrow; i++) {
                    hdata[ncol*i] = (float) rand() / RAND_MAX;
                }
                h_pos[0] -= x_incr;
            }
            if (y_diff > y_incr) {
                shift_terrain_y(hdata, -1, nrow, ncol);
                for (int i = 0; i < nrow; i++) {
                    hdata[ncol*(nrow-1)+i] = (float) rand() / RAND_MAX;
                }
                h_pos[1] += y_incr;
            } else if (y_diff < -y_incr) {
                shift_terrain_y(hdata, 1, nrow, ncol);
                for (int i = 0; i < nrow; i++) {
                    hdata[i] = (float) rand() / RAND_MAX;
                }
                h_pos[1] -= y_incr;
            }
            if ((fabs(x_diff) > x_incr) || (fabs(y_diff) > y_incr)) {
                cassie_sim_set_body_name_pos(c, "floor", h_pos);
                cassie_sim_set_hfielddata(c, hdata);
                cassie_vis_remakeSceneCon(v);
            }
            // h_pos[0] = qpos[0];
            // h_pos[1] = qpos[1];
            // cassie_sim_set_geom_name_pos(c, "hfield1", h_pos);
            // cassie_sim_set_body_name_pos(c, "floor", h_pos);
            // printf("hfield pos %f %f\n", hfield_pos[0], hfield_pos[1]);
        }
        draw_state = cassie_vis_draw(v, c);
        sleep_time = 25000 - (get_microseconds() - start_t);
        if (sleep_time < 0){
            sleep_time = 0;
        }
        usleep(sleep_time);
         
    }

    cassie_sim_free(c);
    
    // cassie_sim_free(c2);
    printf("done free\n");
    cassie_vis_free(v);
    cassie_cleanup();

    return 0;
}
