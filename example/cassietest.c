#include "cassiemujoco.h"


int main(void)
{
    cassie_sim_t *c = cassie_sim_init();
    cassie_vis_t *v = cassie_vis_init();

    state_out_t y;
    pd_in_t u = {0};

    do {
        cassie_sim_step_pd(c, &y, &u);
    } while (cassie_vis_draw(v, c));

    return 0;
}
