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

static long get_microseconds(void) {
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return now.tv_sec * 1000000 + now.tv_nsec / 1000;
}

int main(void)
{
    const char modelfile[] = "../model/cassie_hfield.xml";
    cassie_sim_t *c = cassie_sim_init(modelfile, false);
    cassie_vis_t *v = cassie_vis_init(c, modelfile, false);

    state_out_t y;
    pd_in_t u = {0};

    // Get size of height field data and set to random data
    // Can just grab hfield pointer and directly set data
    int nhfielddata = cassie_sim_get_nhfielddata(c);
    float* hfield_data = cassie_sim_hfielddata(c);    
    srand(time(NULL));
    for (int i = 0; i < nhfielddata; i++) {
        hfield_data[i] = (float)rand()/(float)(RAND_MAX);
    }

    // Set middle where Cassie is initialized to be flat. Indexing starts at negative x negative y
    // (back right corner of the hfield in relation to where Cassie faces foward.)
    int nrow = cassie_sim_get_hfield_nrow(c);
    int ncol = cassie_sim_get_hfield_ncol(c);
    int zero_rad = 5;   // "Radius" around the center to make flat
    for (int i = -zero_rad; i < zero_rad; i++) {
        for (int j = -zero_rad; j < zero_rad; j++) {
            hfield_data[ncol*(nrow/2+i) + ncol/2+j] = 0;
        }
    }

// Alternative way to set hfield data is to make array and pass to 
// set_hfielddata function
#if 0
    int nhfielddata = cassie_sim_get_nhfielddata(c);
    float hfield_data[nhfielddata];    
    srand(time(NULL));
    for (int i = 0; i < nhfielddata; i++) {
        hfield_data[i] = (float)rand()/(float)(RAND_MAX);
    }
    int nrow = cassie_sim_get_hfield_nrow(c);
    int ncol = cassie_sim_get_hfield_ncol(c);
    int zero_rad = 5;   // "Radius" around the center to make flat
    for (int i = -zero_rad; i < zero_rad; i++) {
        for (int j = -zero_rad; j < zero_rad; j++) {
            hfield_data[ncol*(nrow/2+i) + ncol/2+j] = 0;
        }
    }
    cassie_sim_set_hfielddata(c, hfield_data);
#endif

/*
Can also get and modify the hfield size to change max height or size of hfield
All hfield data must be scaled between 0-1, and max height will set the scaling.
Note that you can NOT change the number of columns and rows of hfield data at runtime
*/
#if 0
    double* hfield_size = cassie_sim_get_hfield_size(c);
    // hfield size is 4 long vector of (x, y, z_top, z_bottom)
    hfield_size[0] *= 0.5;   // Shrink x 
    hfield_size[1] *= 1.5;   // Stretch y
    hfield_size[2] *= 2;     // Increase maximum height
    hfield_size[3] *= 2;     // Increase depth or "thickness" of box

    // Again can also use set_hfield_size function
    // double hfield_size[4] = {50, 25, .5, 0.5};
    // cassie_sim_set_hfield_size(c, hfield_size);
#endif

    // Need to remake mjvScene and mjrContext in order for visualization to reflect hfield changes
    // since cassie_vis was already made. This is not necessary if you modify hfield before cassie_vis is made
    cassie_vis_remakeSceneCon(v);
    
    bool draw_state = cassie_vis_draw(v, c);;
    long start_t = get_microseconds();
    long sleep_time = 0;
    while (draw_state) {
        start_t = get_microseconds();
        if (!cassie_vis_paused(v)) {   
            // Base 40 fps
            for (int i = 0; i < 50; i++) {
                cassie_sim_step_pd(c, &y, &u);
            }
        }
        draw_state = cassie_vis_draw(v, c);
        sleep_time = 25000 - (get_microseconds() - start_t);
        if (sleep_time < 0){
            sleep_time = 0;
        }
        usleep(sleep_time);
    }

    cassie_vis_free(v);
    cassie_sim_free(c);    
    cassie_cleanup();

    return 0;
}
