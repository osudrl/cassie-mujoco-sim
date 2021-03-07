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
    const char modelfile[] = "../model/cassie_mass.xml";
    cassie_sim_t *c = cassie_sim_init(modelfile, false);
    // cassie_sim_t *c2 = cassie_sim_init(modelfile, false);

    // printf("%p\n", cassie_sim_joint_filter(c));
    // printf("%p\n", cassie_sim_joint_filter(c2));
    cassie_vis_t *v = cassie_vis_init(c, modelfile);

    state_out_t y;
    // state_out_t y2;
    pd_in_t u = {0};
    // for (int i = 0; i < 5; i++) {
    //     cassie_sim_step_pd(c, &y, &u);
    // }
    // cassie_out_t test_out;
    // test_out = cassie_sim_get_cassie_out(c);
    // printf("test: %f\n", test_out.leftLeg.hipRollDrive.position);
    // test_out.leftLeg.hipRollDrive.position = 1.0;
    // test_out = cassie_sim_get_cassie_out(c);
    // printf("test after: %f\n", test_out.leftLeg.hipRollDrive.position);
    // cassie_sim_copy(c2,  c);
    // cassie_sim_step_pd(c, &y, &u);
    // cassie_sim_step_pd(c2, &y2, &u);
    // double p_diff = 0;
    // for (int i = 0; i < 3; i++) {
    //     p_diff += abs(y.pelvis.position[i] - y2.pelvis.position[i]);
    // }
    
    // state_out_t* est1 = calloc(1, sizeof(state_out_t));
    // est1 = state_output_alloc();
    // state_output_setup(est1);
    // state_out_t* est_p = cassie_sim_get_state_est_p(c);
    // printf("%p\n", cassie_sim_gset_state_est_p(c));
    // printf("%f\n", est_p->pelvis.position[0]);
    // state_out_t est1;
    // printf("alloc\n");
    // cassie_sim_get_state_est(c, &est1);
    // // printf("%d", est1);
    // printf("after get\n");
    // cassie_sim_copy_state_est(c2, &est1);
    // printf("after copy\n");
    // printf("pel pos diff: %f\n", p_diff);


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
    
    bool draw_state = cassie_vis_draw(v, c);
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
        
    }

    cassie_sim_free(c);
    // cassie_sim_free(c2);
    printf("done free\n");
    // cassie_vis_free(v);
    // do {
    //     if (!cassie_vis_paused(v)) {    
    //         cassie_sim_step_pd(c, &y, &u);
    //     }
    // } while (cassie_vis_draw(v, c));

    return 0;
}
