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

#ifndef CASSIECORESIM_H
#define CASSIECORESIM_H

#include "cassie_user_in_t.h"
#include "cassie_out_t.h"
#include "cassie_in_t.h"

typedef struct CassieCoreSim CassieCoreSim;

#ifdef __cplusplus
extern "C" {
#endif

CassieCoreSim* CassieCoreSim_alloc(void);
void CassieCoreSim_copy(CassieCoreSim *dst, const CassieCoreSim *src);
void CassieCoreSim_free(CassieCoreSim *sys);
void CassieCoreSim_setup(CassieCoreSim *sys);
void CassieCoreSim_step(CassieCoreSim *sys, const cassie_user_in_t *in1,
  const cassie_out_t *in2, cassie_in_t *out1);

#ifdef __cplusplus
}
#endif
#endif // CASSIECORESIM_H
