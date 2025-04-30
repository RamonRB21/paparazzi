/*
 * Copyright (C) 2023 Florian Sansou <florian.sansou@enac.fr>
 * Copyright (C) 2023 Gautier Hattenberger <gautier.hattenberger@enac.fr>
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/** @file "modules/ctrl/eff_scheduling_heewing.c"
 * The control effectiveness scheduler for the Heewing T1 Ranger VTOL
 */

#include "modules/ctrl/eff_scheduling_heewing.h"

#include "generated/airframe.h"
#include "state.h"

#include "firmwares/rotorcraft/stabilization/stabilization_indi.h"
#include "firmwares/rotorcraft/guidance/guidance_indi_hybrid.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/guidance.h"

#include "generated/modules.h"
#include "std.h"
#include "modules/core/commands.h"
#include "modules/actuators/actuators.h"

//CHECK IF THIS INCLUDES ARE NEEDED
#include "modules/core/abi.h"
#include "filters/low_pass_filter.h"
#include "modules/radio_control/radio_control.h"


// Left Motor Roll Effectiveness
#ifndef ROLL_EFF_ML
#define ROLL_EFF_ML 1
#endif

// Right Motor Roll Effectiveness
#ifndef ROLL_EFF_MR
#define ROLL_EFF_MR 1
#endif

// Left Tilt Roll Effectiveness
#ifndef ROLL_EFF_LT
#define ROLL_EFF_LT 1
#endif

// Right Tilt Roll Effectiveness
#ifndef ROLL_EFF_RT
#define ROLL_EFF_RT 1
#endif

// Left Motor Pitch Effectiveness
#ifndef PITCH_EFF_ML
#define PITCH_EFF_ML 1
#endif

// Right Motor Pitch Effectiveness
#ifndef PITCH_EFF_MR
#define PITCH_EFF_MR 1
#endif

// Back Motor Pitch Effectiveness
#ifndef PITCH_EFF_MB
#define PITCH_EFF_MB 1
#endif

// Left Tilt Pitch Effectiveness
#ifndef PITCH_EFF_LT
#define PITCH_EFF_LT 1
#endif

// Right Tilt Pitch Effectiveness
#ifndef PITCH_EFF_RT
#define PITCH_EFF_RT 1
#endif

// Left Motor Yaw Effectiveness
#ifndef YAW_EFF_ML
#define YAW_EFF_ML 1
#endif

// Right Motor Yaw Effectiveness
#ifndef YAW_EFF_MR
#define YAW_EFF_MR 1
#endif

// Left Tilt Yaw Effectiveness
#ifndef YAW_EFF_LT
#define YAW_EFF_LT 1
#endif

// Right Tilt Yaw Effectiveness
#ifndef YAW_EFF_RT
#define YAW_EFF_RT 1
#endif

// Left Motor Thrust Effectiveness
#ifndef THRUST_EFF_ML
#define THRUST_EFF_ML 1
#endif

// Right Motor Thrust Effectiveness
#ifndef THRUST_EFF_MR
#define THRUST_EFF_MR 1
#endif

// Left Tilt Thrust Effectiveness
#ifndef THRUST_EFF_LT
#define THRUST_EFF_LT 1
#endif

// Right Tilt Thrust Effectiveness
#ifndef THRUST_EFF_RT
#define THRUST_EFF_RT 1
#endif

// Left Motor Lift Effectiveness
#ifndef LIFT_EFF_ML
#define LIFT_EFF_ML 1
#endif

// Right Motor Lift Effectiveness
#ifndef LIFT_EFF_MR
#define LIFT_EFF_MR 1
#endif

// Back Motor Lift Effectiveness
#ifndef LIFT_EFF_MB
#define LIFT_EFF_MB 1
#endif

// Left Tilt Lift Effectiveness
#ifndef LIFT_EFF_LT
#define LIFT_EFF_LT 1
#endif

// Right Tilt Lift Effectiveness
#ifndef LIFT_EFF_RT
#define LIFT_EFF_RT 1
#endif

// Motor idle position
#ifndef CMH_MOTOR_IDLE
#define CMH_MOTOR_IDLE 800
#endif

/* Effectiveness Matrix definition */
float G2_T1[ESH_EFF_MAT_COLS_NB]                           = {0}; //scaled by G_SCALE
float G1_T1[ESH_EFF_MAT_ROWS_NB][ESH_EFF_MAT_COLS_NB]      = {0}; //scaled by G_SCALE 
float G1G2_T1[ESH_EFF_MAT_ROWS_NB + 1][ESH_EFF_MAT_COLS_NB]      = {0}; //scaled by G_SCALE 

float ctrl_surfaces_eff = 1;

struct FloatEulers eulers_zxy_ESH;

struct T1_Model T1;

inline void eff_scheduling_heewing_update_tilt_angle(void);
inline void eff_scheduling_heewing_update_airspeed(void);
inline void eff_scheduling_heewing_update_thrust(void);
void  update_attitude(void);
void  sum_copy_EFF_MAT(void);
void  init_T1_Model(void);
void  calc_G1_G2(void); 
void stabilization_indi_set_wls_settings(void);
inline void guidance_indi_hybrid_set_wls_settings(float body_v[3], float roll_angle, float pitch_angle);

void eff_scheduling_heewing_init(void)
{  
  init_T1_Model();
  update_attitude();
}

void init_T1_Model(void)
{ 
  T1.lift_eff_ml = LIFT_EFF_ML;
  T1.lift_eff_mr = LIFT_EFF_MR;
  T1.lift_eff_mb = LIFT_EFF_MB;
  T1.lift_eff_lt = LIFT_EFF_LT;
  T1.lift_eff_rt = LIFT_EFF_RT;
  T1.thrust_eff_ml = THRUST_EFF_ML;
  T1.thrust_eff_mr = THRUST_EFF_MR;
  T1.thrust_eff_lt = THRUST_EFF_LT;
  T1.thrust_eff_rt = THRUST_EFF_RT;
  
  // Inertia
  T1.I_XX = I_XX_0TILT;
  T1.I_YY = I_YY_0TILT; 
  T1.I_ZZ = I_ZZ_0TILT; 

  T1.aero_coeff = 0.0668; // 0.5 * density (1.22) * wing area (0.11)
  T1.chord = 0.15;
  T1.span = 0.73;
  T1.mL.dX = 0.09; // Lever arm (absolute value)
  T1.mL.dY = 0.12; // Lever arm (absolute value)
  T1.mR.dX = 0.09; // Lever arm (absolute value)
  T1.mR.dY = 0.12; // Lever arm (absolute value)
  T1.mB.dX = 0.25; // Lever arm (absolute value)
  T1.mB.dY = 0.0;  // Lever arm (absolute value)
}

void update_attitude(void)
{
  float_eulers_of_quat_zxy(&eulers_zxy_ESH, stateGetNedToBodyQuat_f());
  T1.att.phi    = eulers_zxy_ESH.phi;
  T1.att.theta  = eulers_zxy_ESH.theta;
  T1.att.psi    = eulers_zxy_ESH.psi;
  T1.att.sphi   = sinf(eulers_zxy_ESH.phi);
  T1.att.cphi   = cosf(eulers_zxy_ESH.phi);
  T1.att.stheta = sinf(eulers_zxy_ESH.theta);
  T1.att.ctheta = cosf(eulers_zxy_ESH.theta);
  T1.att.spsi   = sinf(eulers_zxy_ESH.psi);
  T1.att.cpsi   = cosf(eulers_zxy_ESH.psi);
}

void eff_scheduling_heewing_periodic(void)
{
  update_attitude();
  eff_scheduling_heewing_update_tilt_angle();
  eff_scheduling_heewing_update_thrust();
  eff_scheduling_heewing_update_airspeed();
  calc_G1_G2();
  sum_copy_EFF_MAT();
}

void eff_scheduling_heewing_update_tilt_angle(void)
{
  T1.tiltl.rad  = 1.73329 * (actuator_state_filt_vect[ESH_CMD_MOTORLT] + MAX_PPRZ) / (2. * MAX_PPRZ);
  Bound(T1.tiltl.rad, 0. , 1.73329)
  T1.tiltl.deg   = T1.tiltl.rad / M_PI * 180.;
  T1.tiltl.cosr  = cosf(T1.tiltl.rad);
  T1.tiltl.sinr  = sinf(T1.tiltl.rad);
  T1.tiltr.rad  = 1.73329 * (actuator_state_filt_vect[ESH_CMD_MOTORRT] + MAX_PPRZ) / (2. * MAX_PPRZ);
  Bound(T1.tiltr.rad, 0. , 1.73329)
  T1.tiltr.deg   = T1.tiltr.rad / M_PI * 180.;
  T1.tiltr.cosr  = cosf(T1.tiltr.rad);
  T1.tiltr.sinr  = sinf(T1.tiltr.rad);
}

void eff_scheduling_heewing_update_thrust(void)
{
  //T1.mL.T = 3.5 * actuator_state_filt_vect[ESH_CMD_MOTORL] / MAX_PPRZ;
  //Bound(T1.mL.T, 0. , 3.5)
  //T1.mR.T = 3.5 * actuator_state_filt_vect[ESH_CMD_MOTORR] / MAX_PPRZ;
  //Bound(T1.mR.T, 0. , 3.5)
  //T1.mB.T = 3.5 * actuator_state_filt_vect[ESH_CMD_MOTORB] / MAX_PPRZ;
  //Bound(T1.mB.T, 0. , 3.5)

  T1.mL.T = actuator_state_filt_vect[ESH_CMD_MOTORL];
  T1.mR.T = actuator_state_filt_vect[ESH_CMD_MOTORR];
  T1.mB.T = actuator_state_filt_vect[ESH_CMD_MOTORB];
}

void eff_scheduling_heewing_update_airspeed(void)
{
  T1.as = stateGetAirspeed_f();
  Bound(T1.as, 0. , 30.);
  T1.as2 = T1.as * T1.as;
  Bound(T1.as2, 0. , 900.);
}

void calc_G1_G2(void)
{
  //printf("Vel: %f, Lsin: %f, Rsin: %f, LThrust: %f, RThrust: %f, BThrust: %f\n", T1.as, T1.tiltl.sinr, T1.tiltr.sinr, T1.mL.T, T1.mR.T, T1.mB.T);
  // Motor Left
  G1_T1[ESH_P][ESH_CMD_MOTORL]  =  (ROLL_EFF_ML   * T1.tiltl.sinr * T1.mL.dY) / T1.I_XX;
  G1_T1[ESH_Q][ESH_CMD_MOTORL]  =  (PITCH_EFF_ML  * T1.tiltl.sinr * T1.mL.dX) / T1.I_YY;
  G1_T1[ESH_R][ESH_CMD_MOTORL]  =  (YAW_EFF_ML    * T1.tiltl.cosr * T1.mL.dY) / T1.I_ZZ;
  G1_T1[ESH_W][ESH_CMD_MOTORL]  = -(T1.lift_eff_ml   * T1.tiltl.sinr) / MASS;
  G1_T1[ESH_U][ESH_CMD_MOTORL]  =  (T1.thrust_eff_ml * T1.tiltl.cosr) / MASS;
  G2_T1[ESH_CMD_MOTORL]         =  0; //T1.mL.dMdud / T1.I_ZZ;
  // Motor Right
  G1_T1[ESH_P][ESH_CMD_MOTORR]  = -(ROLL_EFF_MR   * T1.tiltr.sinr * T1.mR.dY) / T1.I_XX;
  G1_T1[ESH_Q][ESH_CMD_MOTORR]  =  (PITCH_EFF_MR  * T1.tiltr.sinr * T1.mR.dX) / T1.I_YY;
  G1_T1[ESH_R][ESH_CMD_MOTORR]  = -(YAW_EFF_MR    * T1.tiltr.cosr * T1.mR.dY) / T1.I_ZZ;
  G1_T1[ESH_W][ESH_CMD_MOTORR]  = -(T1.lift_eff_mr   * T1.tiltr.sinr) / MASS;
  G1_T1[ESH_U][ESH_CMD_MOTORR]  =  (T1.thrust_eff_mr * T1.tiltr.cosr) / MASS;
  G2_T1[ESH_CMD_MOTORR]         =  0; //T1.mR.dMdud / T1.I_ZZ;
  // Motor Back
  G1_T1[ESH_Q][ESH_CMD_MOTORB]  = -(PITCH_EFF_MB * T1.mB.dX) / T1.I_YY;
  G1_T1[ESH_W][ESH_CMD_MOTORB]  = -T1.lift_eff_mb / MASS;
  G2_T1[ESH_CMD_MOTORB]         =  0; //-T1.mB.dMdud / T1.I_ZZ;
  // Motor Left Tilt
  G1_T1[ESH_P][ESH_CMD_MOTORLT]  =  (ROLL_EFF_LT   * T1.mL.T * T1.tiltl.cosr * T1.mL.dY) / T1.I_XX;
  G1_T1[ESH_Q][ESH_CMD_MOTORLT]  =  (PITCH_EFF_LT  * T1.mL.T * T1.tiltl.cosr * T1.mL.dX) / T1.I_YY;
  G1_T1[ESH_R][ESH_CMD_MOTORLT]  = -(YAW_EFF_LT    * T1.mL.T * T1.tiltl.sinr * T1.mL.dY) / T1.I_ZZ;
  G1_T1[ESH_W][ESH_CMD_MOTORLT]  = -(T1.lift_eff_lt   * T1.mL.T * T1.tiltl.cosr) / MASS;
  G1_T1[ESH_U][ESH_CMD_MOTORLT]  = -(T1.thrust_eff_lt * T1.mL.T * T1.tiltl.sinr) / MASS;
  G2_T1[ESH_CMD_MOTORLT]         =  0; //T1.mR.dMdud / T1.I_ZZ;
  // Motor Right Tilt
  G1_T1[ESH_P][ESH_CMD_MOTORRT]  = -(ROLL_EFF_RT   * T1.mR.T * T1.tiltr.cosr * T1.mR.dY) / T1.I_XX;
  G1_T1[ESH_Q][ESH_CMD_MOTORRT]  =  (PITCH_EFF_RT  * T1.mR.T * T1.tiltr.cosr * T1.mR.dX) / T1.I_YY;
  G1_T1[ESH_R][ESH_CMD_MOTORRT]  =  (YAW_EFF_RT    * T1.mR.T * T1.tiltr.sinr * T1.mR.dY) / T1.I_ZZ;
  G1_T1[ESH_W][ESH_CMD_MOTORRT]  = -(T1.lift_eff_rt   * T1.mR.T * T1.tiltr.cosr) / MASS;
  G1_T1[ESH_U][ESH_CMD_MOTORRT]  = -(T1.thrust_eff_rt * T1.mR.T * T1.tiltr.sinr) / MASS;
  G2_T1[ESH_CMD_MOTORRT]         =  0; //T1.mR.dMdud / T1.I_ZZ;
  // Criteria to not saturate control surfaces at low speed and start using them at around 10 m/s
  if (T1.as < 10) {
    ctrl_surfaces_eff = 0; //-9.9 * T1.as + 100; // VERIFY IF IT WORKS AT VERY LOW SPEEDS (near 0)
  } else {
    ctrl_surfaces_eff = 1;
  }
  // Aileron
  G1_T1[ESH_P][ESH_CMD_AILERONS] = ctrl_surfaces_eff * (T1.aero_coeff * T1.as2 * T1.span * ROLL_D2_AILERONS) / T1.I_XX;
  // ZEROED FOR NOW (ailerons don't contribute that much to yaw)
  //G1_T1[ESH_R][ESH_CMD_AILERONS] = ctrl_surfaces_eff * (T1.aero_coeff * T1.as2 * T1.span * YAW_D2_AILERONS)  / T1.I_ZZ;
  // Elevator
  G1_T1[ESH_Q][ESH_CMD_ELEVATOR] = ctrl_surfaces_eff * (T1.aero_coeff * T1.as2 * T1.chord * PITCH_D2_ELEVATOR) / T1.I_YY;
}

// ZXY Rotation to allocate gimbal lock at 90deg roll (low probability)
void sum_copy_EFF_MAT(void) {
  #if 0
  for (int8_t i = 0; i < ESH_EFF_MAT_COLS_NB; i++) {
    switch (i) {
    case (ESH_CMD_MOTORL):
    case (ESH_CMD_MOTORR):  
    case (ESH_CMD_MOTORLT):
    case (ESH_CMD_MOTORRT):  
      G1G2_T1[ESH_PHI][i] = G1_T1[ESH_P][i] + (T1.att.sphi * T1.att.stheta / T1.att.ctheta) * G1_T1[ESH_Q][i] + (T1.att.cphi * T1.att.stheta / T1.att.ctheta) * (G1_T1[ESH_R][i] + G2_T1[i]);
      G1G2_T1[ESH_THE][i] = (T1.att.cphi                                                  ) * G1_T1[ESH_Q][i] + (-T1.att.sphi                               ) * (G1_T1[ESH_R][i] + G2_T1[i]);
      G1G2_T1[ESH_PSI][i] = (T1.att.sphi / T1.att.ctheta                                  ) * G1_T1[ESH_Q][i] + (T1.att.cphi / T1.att.ctheta                ) * (G1_T1[ESH_R][i] + G2_T1[i]);
      G1G2_T1[ESH_D][i]   = (-T1.att.cphi * T1.att.stheta                                           ) * G1_T1[ESH_U][i] + (T1.att.cphi * T1.att.ctheta                                            ) * G1_T1[ESH_W][i];
      G1G2_T1[ESH_N][i]   = (T1.att.cpsi * T1.att.ctheta - T1.att.spsi * T1.att.sphi * T1.att.stheta) * G1_T1[ESH_U][i] + (T1.att.cpsi * T1.att.stheta + T1.att.spsi * T1.att.sphi * T1.att.ctheta) * G1_T1[ESH_W][i];
      //G1G2_T1[ESH_E][i]   = (T1.att.spsi * T1.att.ctheta + T1.att.cpsi * T1.att.sphi * T1.att.stheta) * G1_T1[ESH_U][i] + (T1.att.spsi * T1.att.stheta - T1.att.cpsi * T1.att.sphi * T1.att.ctheta) * G1_T1[ESH_W][i];
      break;
    case (ESH_CMD_MOTORB):  
      G1G2_T1[ESH_PHI][i] = (T1.att.sphi * T1.att.stheta / T1.att.ctheta) * G1_T1[ESH_Q][i] + (T1.att.cphi * T1.att.stheta / T1.att.ctheta) * G2_T1[i];
      G1G2_T1[ESH_THE][i] = (T1.att.cphi                                ) * G1_T1[ESH_Q][i] + (-T1.att.sphi                               ) * G2_T1[i];
      G1G2_T1[ESH_PSI][i] = (T1.att.sphi / T1.att.ctheta                ) * G1_T1[ESH_Q][i] + (T1.att.cphi / T1.att.ctheta                ) * G2_T1[i];
      G1G2_T1[ESH_D][i]   = (T1.att.cphi * T1.att.ctheta                                            ) * G1_T1[ESH_W][i];
      G1G2_T1[ESH_N][i]   = (T1.att.cpsi * T1.att.stheta + T1.att.spsi * T1.att.sphi * T1.att.ctheta) * G1_T1[ESH_W][i];
      //G1G2_T1[ESH_E][i]   = (T1.att.spsi * T1.att.stheta - T1.att.cpsi * T1.att.sphi * T1.att.ctheta) * G1_T1[ESH_W][i];
      break;
    case (ESH_CMD_AILERONS):
      G1G2_T1[ESH_PHI][i] = G1_T1[ESH_P][i] + (T1.att.cphi * T1.att.stheta / T1.att.ctheta) * (G1_T1[ESH_R][i] + G2_T1[i]);
      G1G2_T1[ESH_THE][i] =                   (-T1.att.sphi                               ) * (G1_T1[ESH_R][i] + G2_T1[i]);
      G1G2_T1[ESH_PSI][i] =                   (T1.att.cphi / T1.att.ctheta                ) * (G1_T1[ESH_R][i] + G2_T1[i]);
      G1G2_T1[ESH_D][i]   = 0.0;
      G1G2_T1[ESH_N][i]   = 0.0;
      //G1G2_T1[ESH_E][i]   = 0.0;
      break; 
    case (ESH_CMD_ELEVATOR): 
      G1G2_T1[ESH_PHI][i] = (T1.att.sphi * T1.att.stheta / T1.att.ctheta) * G1_T1[ESH_Q][i];
      G1G2_T1[ESH_THE][i] = (T1.att.cphi                                ) * G1_T1[ESH_Q][i];
      G1G2_T1[ESH_PSI][i] = (T1.att.sphi / T1.att.ctheta                ) * G1_T1[ESH_Q][i];
      G1G2_T1[ESH_D][i]   = 0.0;
      G1G2_T1[ESH_N][i]   = 0.0;
      //G1G2_T1[ESH_E][i]   = 0.0;
      break;
    default:
      break;
    }
  }
  
  for (i = 0; i < ESH_EFF_MAT_ROWS_NB; i++) {
    for (j = 0; j < ESH_EFF_MAT_COLS_NB; j++) {
      float abs = fabs(G1G2_T1[i][j]);
      switch (i) {
        case (ESH_N):
        case (ESH_E):
        case (ESH_D):
        case (ESH_PHI):
        case (ESH_THE):
        case (ESH_PSI):
          if (abs < FLT_CUTOFF_FREQ) {
            G1G2_T1[i][j] = 0.0;
          }
          break;
      }
    }
  }
  #endif
  for (int8_t i = 0; i < INDI_OUTPUTS; i++) {
    for (int8_t j = 0; j < INDI_NUM_ACT; j++) {
        G1G2_T1[i][j] = G1_T1[i][j];
        g1g2[i][j] = G1_T1[i][j] / INDI_G_SCALING;
    }
  }
}

void stabilization_indi_set_wls_settings(void)
{
  for (int8_t i = 0; i < ESH_EFF_MAT_COLS_NB; i++) {
    switch (i) {
      case (ESH_CMD_MOTORL):
      case (ESH_CMD_MOTORR):
      case (ESH_CMD_MOTORB):
        wls_stab_p.u_min[i] = CMH_MOTOR_IDLE;
        wls_stab_p.u_max[i] = MAX_PPRZ;
        wls_stab_p.u_pref[i] = act_pref[i];
        break;
      case(ESH_CMD_MOTORLT):
      case(ESH_CMD_MOTORRT):
      case(ESH_CMD_AILERONS):
      case(ESH_CMD_ELEVATOR):
        wls_stab_p.u_min[i] = -MAX_PPRZ;
        wls_stab_p.u_max[i] = MAX_PPRZ;
        wls_stab_p.u_pref[i] = act_pref[i];
        break;
    default:
      break;
    }
  }
}

//TODO
// Override standard LIFT_D function
float guidance_indi_get_liftd(float pitch UNUSED, float theta UNUSED) {
  return (-T1.aero_coeff * T1.as2 * LIFT_D2_ALPHA);
}

void guidance_indi_hybrid_set_wls_settings(float body_v[3], float roll_angle, float pitch_angle)
{
  // Weights evolution
  float Wu_original[GUIDANCE_INDI_HYBRID_U] = GUIDANCE_INDI_WLS_WU;
  float Wv_original[GUIDANCE_INDI_HYBRID_V] = GUIDANCE_INDI_WLS_PRIORITIES;

  wls_guid_p.Wu[ESH_CMD_PITCH] = Wu_original[ESH_CMD_PITCH] + 5 * T1.as;
  wls_guid_p.Wu[ESH_CMD_FX] = Wu_original[ESH_CMD_FX] + 300 * (1 - pow(M_E, (T1.as / 4.0)));
  wls_guid_p.Wv[0] = Wv_original[0] + 3 * T1.as; //maybe could evolute with tilt angle

  //Set Limits
  float max_pitch_limit_rad = RadOfDeg(GUIDANCE_INDI_MAX_PITCH);
  float min_pitch_limit_rad = RadOfDeg(GUIDANCE_INDI_MIN_PITCH);

  // float pitch_pref_rad = RadOfDeg(guidance_indi_pitch_pref_deg);

  //REVIEW THIS AFTER TESTING
  // Set lower limits
  wls_guid_p.u_min[ESH_CMD_ROLL]  = -guidance_indi_max_bank - roll_angle;
  wls_guid_p.u_min[ESH_CMD_PITCH] =  min_pitch_limit_rad - pitch_angle;
  wls_guid_p.u_min[ESH_CMD_FZ]    = -((MAX_PPRZ) * G1G2_T1[ESH_D][ESH_CMD_MOTORL] + 
                                      (MAX_PPRZ) * G1G2_T1[ESH_D][ESH_CMD_MOTORR] +
                                      (MAX_PPRZ) * G1G2_T1[ESH_D][ESH_CMD_MOTORB]);// +
                                      //(MAX_PPRZ) * G1G2_T1[ESH_D][ESH_CMD_MOTORLT] +
                                      //(MAX_PPRZ) * G1G2_T1[ESH_D][ESH_CMD_MOTORRT]);
  wls_guid_p.u_min[ESH_CMD_FX]    = wls_stab_p.u_min[ESH_CMD_MOTORL] * G1G2_T1[ESH_N][ESH_CMD_MOTORL] + 
                                    wls_stab_p.u_min[ESH_CMD_MOTORR] * G1G2_T1[ESH_N][ESH_CMD_MOTORR] +
                                    wls_stab_p.u_min[ESH_CMD_MOTORB] * G1G2_T1[ESH_N][ESH_CMD_MOTORB];// +
                                    //wls_stab_p.u_min[ESH_CMD_MOTORLT] * G1G2_T1[ESH_N][ESH_CMD_MOTORLT] +
                                    //wls_stab_p.u_min[ESH_CMD_MOTORRT] * G1G2_T1[ESH_N][ESH_CMD_MOTORRT];

  // Set upper limits
  wls_guid_p.u_max[ESH_CMD_ROLL]  =  guidance_indi_max_bank - roll_angle;
  wls_guid_p.u_max[ESH_CMD_PITCH] =  max_pitch_limit_rad - pitch_angle;
  wls_guid_p.u_max[ESH_CMD_FZ]    = -(wls_stab_p.u_min[ESH_CMD_MOTORL] * G1G2_T1[ESH_D][ESH_CMD_MOTORL] + 
                                      wls_stab_p.u_min[ESH_CMD_MOTORR] * G1G2_T1[ESH_D][ESH_CMD_MOTORR] +
                                      wls_stab_p.u_min[ESH_CMD_MOTORB] * G1G2_T1[ESH_D][ESH_CMD_MOTORB]);// +
                                      //wls_stab_p.u_min[ESH_CMD_MOTORLT] * G1G2_T1[ESH_D][ESH_CMD_MOTORLT] +
                                      //wls_stab_p.u_min[ESH_CMD_MOTORRT] * G1G2_T1[ESH_D][ESH_CMD_MOTORRT]);
  wls_guid_p.u_max[ESH_CMD_FX]    = (MAX_PPRZ) * G1G2_T1[ESH_N][ESH_CMD_MOTORL] + 
                                    (MAX_PPRZ) * G1G2_T1[ESH_N][ESH_CMD_MOTORR] +
                                    (MAX_PPRZ) * G1G2_T1[ESH_N][ESH_CMD_MOTORB];// +
                                    //(MAX_PPRZ) * G1G2_T1[ESH_N][ESH_CMD_MOTORLT] +
                                    //(MAX_PPRZ) * G1G2_T1[ESH_N][ESH_CMD_MOTORRT];

  // Set prefered states
  wls_guid_p.u_pref[ESH_CMD_ROLL]  = -roll_angle; // prefered delta roll angle
  wls_guid_p.u_pref[ESH_CMD_PITCH] = -pitch_angle; // + pitch_pref_rad;// prefered delta pitch angle

  wls_guid_p.u_pref[ESH_CMD_FZ]    =  wls_guid_p.u_max[ESH_CMD_FZ]; // Low thrust better for efficiency
  wls_guid_p.u_pref[ESH_CMD_FX]    =  body_v[0]; // solve the body acceleration
}

float get_max_pusher_thrust() {
  return MAX_PPRZ * g1g2[ESH_N][ESH_CMD_MOTORL] + MAX_PPRZ * g1g2[ESH_N][ESH_CMD_MOTORR] +
         MAX_PPRZ * g1g2[ESH_N][ESH_CMD_MOTORB];// + MAX_PPRZ * g1g2[ESH_N][ESH_CMD_MOTORLT] +
         //MAX_PPRZ * g1g2[ESH_N][ESH_CMD_MOTORRT];
}