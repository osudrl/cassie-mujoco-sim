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

#ifndef PD_INPUT_H
#define PD_INPUT_H

#include "pd_in_t.h"
#include "cassie_out_t.h"
#include "cassie_user_in_t.h"

typedef struct PdInput pd_input_t;

#ifdef __cplusplus
extern "C" {
#endif

pd_input_t* pd_input_alloc(void);
void pd_input_copy(pd_input_t *dst, const pd_input_t *src);
void pd_input_free(pd_input_t *sys);
void pd_input_setup(pd_input_t *sys);
void pd_input_step(pd_input_t *sys, const pd_in_t *in1, const cassie_out_t
  *in2, cassie_user_in_t *out1);

#ifdef __cplusplus
}
#endif
#endif // PD_INPUT_H
