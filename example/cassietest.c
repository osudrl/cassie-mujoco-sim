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

int main(void)
{
    const char modelfile[] = "../model/cassie.xml";
    cassie_sim_t *c = cassie_sim_init(modelfile, false);
    // cassie_sim_t *c2 = cassie_sim_init(modelfile, false);

    // printf("%p\n", cassie_sim_joint_filter(c));
    // printf("%p\n", cassie_sim_joint_filter(c2));
    cassie_vis_t *v = cassie_vis_init(c, modelfile, false);
    // cassie_vis_init_depth(v, 300, 300);
    // cassie_vis_full_reset(v);
    cassie_vis_t *v2 = cassie_vis_init(c, modelfile, false);
    cassie_vis_window_resize(v2, 300, 300);
    cassie_vis_init_depth(v2, 300, 300);

    state_out_t y;
    // state_out_t y2;
    pd_in_t u = {0};


    // pd_in_t u2;
    // printf("pd size: %li\n", sizeof(u2));

    
    // int count = 0;
    // double apply_force[6] = {0, 0, 100, 0, 0, 0};
    // float* hfield_data = cassie_sim_hfielddata(c);
    // int nhfielddata = cassie_sim_get_nhfielddata(c);
    // srand(time(NULL));
    // for (int i = 0; i < nhfielddata; i++) {
    //     // printf("%g\n", (float)rand()/(float)RAND_MAX);
    //     hfield_data[i] = (float)rand()/(float)RAND_MAX;
    // }

    for (int i = 0; i < 300; i++) {
        cassie_sim_step_pd(c, &y, &u);
    }
    // 
    bool draw_state = cassie_vis_draw(v, c);
    // cassie_vis_window_resize(v, 300, 300);
    // bool draw_state2 = cassie_vis_draw(v2, c);
    while (draw_state) {
        if (!cassie_vis_paused(v)) {   
            // if (count > 500) {
            //     printf("applying perturb\n");
            //     cassie_vis_apply_force(v, apply_force, "cassie-pelvis");                
            // }
            // printf("count: %i\n", count); 
            cassie_sim_step_pd(c, &y, &u);
            // count += 1;
        }
        draw_state = cassie_vis_draw(v, c);
        cassie_vis_draw_depth(v2, c, 300, 300);
        // draw_state2 = cassie_vis_draw(v2, c);
        
    }

    cassie_sim_free(c);
    
    // cassie_sim_free(c2);
    printf("done free\n");
    cassie_vis_free(v);
    cassie_vis_free(v2);
    cassie_cleanup();

    return 0;
}
