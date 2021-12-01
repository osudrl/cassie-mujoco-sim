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

    state_out_t y;
    pd_in_t u = {0};
        
    bool draw_state = cassie_vis_draw(v, c);
    cassie_vis_init_recording(v, "testFile", 1920, 1080);
    long start_t = get_microseconds();
    long sleep_time = 0;
    while (draw_state) {
        start_t = get_microseconds();
        if (!cassie_vis_paused(v)) {
            for (int i = 0; i < 50; i++) {
                cassie_sim_step_pd(c, &y, &u);
            }
        }
        draw_state = cassie_vis_draw(v, c);
        cassie_vis_record_frame(v);
        sleep_time = 25000 - (get_microseconds() - start_t);
        if (sleep_time < 0){
            sleep_time = 0;
        }
        usleep(sleep_time);
    }

    cassie_vis_close_recording(v);
    cassie_vis_free(v);
    cassie_sim_free(c);
    cassie_cleanup();

    return 0;
}
