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

#ifndef STATE_OUT_T_H
#define STATE_OUT_T_H

#define STATE_OUT_T_PACKED_LEN 493

#include <stdbool.h>

typedef struct {
  double stateOfCharge;
  double current;
} state_battery_out_t;

typedef struct {
  double position[3];
  double orientation[4];
  double footRotationalVelocity[3];
  double footTranslationalVelocity[3];
  double toeForce[3];
  double heelForce[3];
} state_foot_out_t;

typedef struct {
  double position[6];
  double velocity[6];
} state_joint_out_t;

typedef struct {
  double position[10];
  double velocity[10];
  double torque[10];
} state_motor_out_t;

typedef struct {
  double position[3];
  double orientation[4];
  double rotationalVelocity[3];
  double translationalVelocity[3];
  double translationalAcceleration[3];
  double externalMoment[3];
  double externalForce[3];
} state_pelvis_out_t;

typedef struct {
  double channel[16];
  bool signalGood;
} state_radio_out_t;

typedef struct {
  double height;
  double slope[2];
} state_terrain_out_t;

typedef struct {
  state_pelvis_out_t pelvis;
  state_foot_out_t leftFoot;
  state_foot_out_t rightFoot;
  state_terrain_out_t terrain;
  state_motor_out_t motor;
  state_joint_out_t joint;
  state_radio_out_t radio;
  state_battery_out_t battery;
} state_out_t;


#ifdef __cplusplus
extern "C" {
#endif

void pack_state_out_t(const state_out_t *bus, unsigned char *bytes);
void unpack_state_out_t(const unsigned char *bytes, state_out_t *bus);

#ifdef __cplusplus
}
#endif
#endif // STATE_OUT_T_H
