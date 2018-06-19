/*
 * Copyright (c) 2018 Agility Robotics
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

#ifndef CASSIE_CORE_SIM_H
#define CASSIE_CORE_SIM_H

#include "cassie_user_in_t.h"
#include "cassie_out_t.h"
#include "cassie_in_t.h"

typedef struct CassieCoreSim cassie_core_sim_t;

#ifdef __cplusplus
extern "C" {
#endif

cassie_core_sim_t* cassie_core_sim_alloc(void);
void cassie_core_sim_copy(cassie_core_sim_t *dst, const cassie_core_sim_t *src);
void cassie_core_sim_free(cassie_core_sim_t *sys);
void cassie_core_sim_setup(cassie_core_sim_t *sys);
void cassie_core_sim_step(cassie_core_sim_t *sys, const cassie_user_in_t *in1,
  const cassie_out_t *in2, cassie_in_t *out1);

#ifdef __cplusplus
}
#endif
#endif // CASSIE_CORE_SIM_H
