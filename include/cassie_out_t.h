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

#ifndef CASSIE_OUT_T_H
#define CASSIE_OUT_T_H

#define CASSIE_OUT_T_PACKED_LEN 697

#include <stdbool.h>

typedef short DiagnosticCodes;


typedef struct {
  bool dataGood;
  double stateOfCharge;
  double voltage[12];
  double current;
  double temperature[4];
} battery_out_t;

typedef struct {
  double position;
  double velocity;
} cassie_joint_out_t;

typedef struct {
  unsigned short statusWord;
  double position;
  double velocity;
  double torque;
  double driveTemperature;
  double dcLinkVoltage;
  double torqueLimit;
  double gearRatio;
} elmo_out_t;

typedef struct {
  elmo_out_t hipRollDrive;
  elmo_out_t hipYawDrive;
  elmo_out_t hipPitchDrive;
  elmo_out_t kneeDrive;
  elmo_out_t footDrive;
  cassie_joint_out_t shinJoint;
  cassie_joint_out_t tarsusJoint;
  cassie_joint_out_t footJoint;
  unsigned char medullaCounter;
  unsigned short medullaCpuLoad;
  bool reedSwitchState;
} cassie_leg_out_t;

typedef struct {
  bool radioReceiverSignalGood;
  bool receiverMedullaSignalGood;
  double channel[16];
} radio_out_t;

typedef struct {
  int etherCatStatus[6];
  int etherCatNotifications[21];
  double taskExecutionTime;
  unsigned int overloadCounter;
  double cpuTemperature;
} target_pc_out_t;

typedef struct {
  bool dataGood;
  unsigned short vpeStatus;
  double pressure;
  double temperature;
  double magneticField[3];
  double angularVelocity[3];
  double linearAcceleration[3];
  double orientation[4];
} vectornav_out_t;

typedef struct {
  target_pc_out_t targetPc;
  battery_out_t battery;
  radio_out_t radio;
  vectornav_out_t vectorNav;
  unsigned char medullaCounter;
  unsigned short medullaCpuLoad;
  bool bleederState;
  bool leftReedSwitchState;
  bool rightReedSwitchState;
  double vtmTemperature;
} cassie_pelvis_out_t;

typedef struct {
  cassie_pelvis_out_t pelvis;
  cassie_leg_out_t leftLeg;
  cassie_leg_out_t rightLeg;
  bool isCalibrated;
  DiagnosticCodes messages[4];
} cassie_out_t;

#define EMPTY                          ((DiagnosticCodes)0)
#define LEFT_HIP_NOT_CALIB             ((DiagnosticCodes)5)
#define LEFT_KNEE_NOT_CALIB            ((DiagnosticCodes)6)
#define RIGHT_HIP_NOT_CALIB            ((DiagnosticCodes)7)
#define RIGHT_KNEE_NOT_CALIB           ((DiagnosticCodes)8)
#define LOW_BATTERY_CHARGE             ((DiagnosticCodes)200)
#define HIGH_CPU_TEMP                  ((DiagnosticCodes)205)
#define HIGH_VTM_TEMP                  ((DiagnosticCodes)210)
#define HIGH_ELMO_DRIVE_TEMP           ((DiagnosticCodes)215)
#define HIGH_STATOR_TEMP               ((DiagnosticCodes)220)
#define LOW_ELMO_LINK_VOLTAGE          ((DiagnosticCodes)221)
#define HIGH_BATTERY_TEMP              ((DiagnosticCodes)225)
#define RADIO_DATA_BAD                 ((DiagnosticCodes)230)
#define RADIO_SIGNAL_BAD               ((DiagnosticCodes)231)
#define BMS_DATA_BAD                   ((DiagnosticCodes)235)
#define VECTORNAV_DATA_BAD             ((DiagnosticCodes)236)
#define VPE_GYRO_SATURATION            ((DiagnosticCodes)240)
#define VPE_MAG_SATURATION             ((DiagnosticCodes)241)
#define VPE_ACC_SATURATION             ((DiagnosticCodes)242)
#define VPE_ATTITUDE_BAD               ((DiagnosticCodes)245)
#define VPE_ATTITUDE_NOT_TRACKING      ((DiagnosticCodes)246)
#define ETHERCAT_DC_ERROR              ((DiagnosticCodes)400)
#define ETHERCAT_ERROR                 ((DiagnosticCodes)410)
#define LOAD_CALIB_DATA_ERROR          ((DiagnosticCodes)590)
#define CRITICAL_BATTERY_CHARGE        ((DiagnosticCodes)600)
#define CRITICAL_CPU_TEMP              ((DiagnosticCodes)605)
#define CRITICAL_VTM_TEMP              ((DiagnosticCodes)610)
#define CRITICAL_ELMO_DRIVE_TEMP       ((DiagnosticCodes)615)
#define CRITICAL_STATOR_TEMP           ((DiagnosticCodes)620)
#define CRITICAL_BATTERY_TEMP          ((DiagnosticCodes)625)
#define TORQUE_LIMIT_REACHED           ((DiagnosticCodes)630)
#define JOINT_LIMIT_REACHED            ((DiagnosticCodes)635)
#define ENCODER_FAILURE                ((DiagnosticCodes)640)
#define SPRING_FAILURE                 ((DiagnosticCodes)645)
#define LEFT_LEG_MEDULLA_HANG          ((DiagnosticCodes)700)
#define RIGHT_LEG_MEDULLA_HANG         ((DiagnosticCodes)701)
#define PELVIS_MEDULLA_HANG            ((DiagnosticCodes)703)
#define CPU_OVERLOAD                   ((DiagnosticCodes)704)

#ifdef __cplusplus
extern "C" {
#endif

void pack_cassie_out_t(const cassie_out_t *bus, unsigned char *bytes);
void unpack_cassie_out_t(const unsigned char *bytes, cassie_out_t *bus);

#ifdef __cplusplus
}
#endif
#endif // CASSIE_OUT_T_H
