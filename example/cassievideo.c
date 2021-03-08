/*
 * Copyright (c) 2021 Dynamic Robotics Laboratory
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
    const char modelfile[] = "../model/cassie_hfield.xml";
    cassie_sim_t *c = cassie_sim_init(modelfile, false);
    cassie_vis_t *v = cassie_vis_init(c, modelfile);

    state_out_t y;
    pd_in_t u = {0};
    
    int count = 0;
    
    bool draw_state = cassie_vis_draw(v, c);
    cassie_vis_init_recording(v, "testFile", 1920, 1080);
    
    while (draw_state) {

        if (!cassie_vis_paused(v)) {   
            cassie_sim_step_pd(c, &y, &u);
            count += 1;
        }

        if(count%67 == 0){
            draw_state = cassie_vis_draw(v, c);
            cassie_vis_record_frame(v);
        }
        
    }
    cassie_vis_close_recording(v);
    cassie_sim_free(c);
    cassie_vis_free(v);

    return 0;
}
