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

#ifndef PDINPUT_H
#define PDINPUT_H

#include "pd_in_t.h"
#include "cassie_out_t.h"
#include "cassie_user_in_t.h"

typedef struct PdInput PdInput;

#ifdef __cplusplus
extern "C" {
#endif

PdInput* PdInput_alloc(void);
void PdInput_copy(PdInput *dst, const PdInput *src);
void PdInput_free(PdInput *sys);
void PdInput_setup(PdInput *sys);
void PdInput_step(PdInput *sys, const pd_in_t *in1, const cassie_out_t
  *in2, cassie_user_in_t *out1);

#ifdef __cplusplus
}
#endif
#endif // PDINPUT_H
