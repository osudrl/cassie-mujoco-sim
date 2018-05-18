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

#ifndef PD_IN_T_H
#define PD_IN_T_H

#define PD_IN_T_PACKED_LEN 476

#include <stdbool.h>

typedef struct {
  double torque[5];
  double pTarget[5];
  double dTarget[5];
  double pGain[5];
  double dGain[5];
} pd_motor_in_t;

typedef struct {
  double torque[6];
  double pTarget[6];
  double dTarget[6];
  double pGain[6];
  double dGain[6];
} pd_task_in_t;

typedef struct {
  pd_task_in_t taskPd;
  pd_motor_in_t motorPd;
} pd_leg_in_t;

typedef struct {
  pd_leg_in_t leftLeg;
  pd_leg_in_t rightLeg;
  double telemetry[9];
} pd_in_t;


#ifdef __cplusplus
extern "C" {
#endif

void pack_pd_in_t(const pd_in_t *bus, unsigned char *bytes);
void unpack_pd_in_t(const unsigned char *bytes, pd_in_t *bus);

#ifdef __cplusplus
}
#endif
#endif // PD_IN_T_H
