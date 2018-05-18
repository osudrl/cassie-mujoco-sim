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

#ifndef CASSIE_IN_T_H
#define CASSIE_IN_T_H

#define CASSIE_IN_T_PACKED_LEN 91

#include <stdbool.h>

typedef struct {
  unsigned short controlWord;
  double torque;
} elmo_in_t;

typedef struct {
  elmo_in_t hipRollDrive;
  elmo_in_t hipYawDrive;
  elmo_in_t hipPitchDrive;
  elmo_in_t kneeDrive;
  elmo_in_t footDrive;
} cassie_leg_in_t;

typedef struct {
  short channel[14];
} radio_in_t;

typedef struct {
  radio_in_t radio;
  bool sto;
  bool piezoState;
  unsigned char piezoTone;
} cassie_pelvis_in_t;

typedef struct {
  cassie_pelvis_in_t pelvis;
  cassie_leg_in_t leftLeg;
  cassie_leg_in_t rightLeg;
} cassie_in_t;


#ifdef __cplusplus
extern "C" {
#endif

void pack_cassie_in_t(const cassie_in_t *bus, unsigned char *bytes);
void unpack_cassie_in_t(const unsigned char *bytes, cassie_in_t *bus);

#ifdef __cplusplus
}
#endif
#endif // CASSIE_IN_T_H
