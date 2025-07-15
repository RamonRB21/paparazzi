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


// Right Motor Roll Effectiveness
#ifndef ROLL_EFF_MR
#define ROLL_EFF_MR 1
#endif

// Left Motor Roll Effectiveness
#ifndef ROLL_EFF_ML
#define ROLL_EFF_ML 1
#endif

// Right Tilt Roll Effectiveness
#ifndef ROLL_EFF_RT
#define ROLL_EFF_RT 1
#endif

// Left Tilt Roll Effectiveness
#ifndef ROLL_EFF_LT
#define ROLL_EFF_LT 1
#endif

// Ailerons Roll Effectiveness
#ifndef ROLL_EFF_AIL
#define ROLL_EFF_AIL 1
#endif

// Right Motor Pitch Effectiveness
#ifndef PITCH_EFF_MR
#define PITCH_EFF_MR 1
#endif

// Left Motor Pitch Effectiveness
#ifndef PITCH_EFF_ML
#define PITCH_EFF_ML 1
#endif

// Back Motor Pitch Effectiveness
#ifndef PITCH_EFF_MB
#define PITCH_EFF_MB 1
#endif

// Right Tilt Pitch Effectiveness
#ifndef PITCH_EFF_RT
#define PITCH_EFF_RT 1
#endif

// Left Tilt Pitch Effectiveness
#ifndef PITCH_EFF_LT
#define PITCH_EFF_LT 1
#endif

// Elevator Pitch Effectiveness
#ifndef PITCH_EFF_ELE
#define PITCH_EFF_ELE 1
#endif

// Right Motor Yaw Effectiveness
#ifndef YAW_EFF_MR
#define YAW_EFF_MR 1
#endif

// Left Motor Yaw Effectiveness
#ifndef YAW_EFF_ML
#define YAW_EFF_ML 1
#endif

// Right Tilt Yaw Effectiveness
#ifndef YAW_EFF_RT
#define YAW_EFF_RT 1
#endif

// Left Tilt Yaw Effectiveness
#ifndef YAW_EFF_TD
#define YAW_EFF_TD 1
#endif

// Right Motor Thrust Effectiveness
#ifndef THRUST_EFF_MR
#define THRUST_EFF_MR 1
#endif

// Left Motor Thrust Effectiveness
#ifndef THRUST_EFF_ML
#define THRUST_EFF_ML 1
#endif

// Right Tilt Thrust Effectiveness
#ifndef THRUST_EFF_MT
#define THRUST_EFF_MT 1
#endif

// Left Tilt Thrust Effectiveness
#ifndef THRUST_EFF_LT
#define THRUST_EFF_LT 1
#endif

// Right Motor Lift Effectiveness
#ifndef LIFT_EFF_MR
#define LIFT_EFF_MR 1
#endif

// Left Motor Lift Effectiveness
#ifndef LIFT_EFF_ML
#define LIFT_EFF_ML 1
#endif

// Back Motor Lift Effectiveness
#ifndef LIFT_EFF_MB
#define LIFT_EFF_MB 1
#endif

// Right Tilt Lift Effectiveness
#ifndef LIFT_EFF_MT
#define LIFT_EFF_MT 1
#endif

// Left Tilt Lift Effectiveness
#ifndef LIFT_EFF_LT
#define LIFT_EFF_LT 1
#endif

// Motor idle position
#ifndef ESH_MOTOR_IDLE
#define ESH_MOTOR_IDLE 800
#endif

// Max tilt diff
#ifndef ESH_TILT_DIFF_MAX
#define ESH_TILT_DIFF_MAX 900
#endif

// WLS Roll Weight
#ifndef ESH_WLS_ROLL
#define ESH_WLS_ROLL 1000
#endif

// WLS Pitch Weight
#ifndef ESH_WLS_PITCH
#define ESH_WLS_PITCH 1000
#endif

// WLS Yaw Weight
#ifndef ESH_WLS_YAW
#define ESH_WLS_YAW 10
#endif

// WLS Lift Weight
#ifndef ESH_WLS_LIFT
#define ESH_WLS_LIFT 200
#endif

// WLS Thrust Weight
#ifndef ESH_WLS_THRUST
#define ESH_WLS_THRUST 100
#endif

// WLS 3 Motors Weight
#ifndef ESH_WLS_MOTORS
#define ESH_WLS_MOTORS 1.3
#endif

// WLS Mean Tilt Weight
#ifndef ESH_WLS_MEANT
#define ESH_WLS_MEANT 2.0
#endif

// WLS Tilt Diff Weight
#ifndef ESH_WLS_TDIFF
#define ESH_WLS_TDIFF 1.0
#endif

// WLS Tilt Diff Weight
#ifndef ESH_GAMMA_SQ
#define ESH_GAMMA_SQ 10000.0
#endif

// WLS Tilt Diff Weight
#ifndef ESH_WLS_MIN_MT
#define ESH_WLS_MIN_MT 0.0
#endif

// WLS Guidance Lift Efficiency scaling
#ifndef GUIDANCE_ESH_LIFT_EFF
#define GUIDANCE_ESH_LIFT_EFF 1.0
#endif

// Speed at which propellers can go full forward
#ifndef ESH_MIN_MT_SPEED
#define ESH_MIN_MT_SPEED 7.0
#endif

// Minimum Mean Tilt at hovering
#ifndef ESH_MIN_MT_HOVER
#define ESH_MIN_MT_HOVER 8000
#endif

/* Effectiveness Matrix definition */
float G2_T1[ESH_EFF_MAT_COLS_NB]                           = {0}; //scaled by G_SCALE
float G1_T1[ESH_EFF_MAT_ROWS_NB][ESH_EFF_MAT_COLS_NB]      = {0}; //scaled by G_SCALE 
float G1G2_T1[ESH_EFF_MAT_ROWS_NB + 1][ESH_EFF_MAT_COLS_NB]      = {0}; //scaled by G_SCALE 

float ctrl_surfaces_eff = 1;
float esh_counter;

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

void eff_scheduling_heewing_init(void)
{  
  init_T1_Model();
  update_attitude();
}

void init_T1_Model(void)
{ 
  T1.roll_eff_ail = ROLL_EFF_AIL;
  T1.pitch_eff_ele = PITCH_EFF_ELE;
  T1.pitch_eff_mb = PITCH_EFF_MB;
  T1.yaw_eff_mr = YAW_EFF_MR;
  T1.yaw_eff_ml = YAW_EFF_ML;
  T1.yaw_eff_td = YAW_EFF_TD;
  T1.lift_eff_mr = LIFT_EFF_MR;
  T1.lift_eff_ml = LIFT_EFF_ML;
  T1.lift_eff_mb = LIFT_EFF_MB;
  T1.lift_eff_mt = LIFT_EFF_MT;
  T1.thrust_eff_mr = THRUST_EFF_MR;
  T1.thrust_eff_ml = THRUST_EFF_ML;
  T1.thrust_eff_mt = THRUST_EFF_MT;

  T1.wls_roll = ESH_WLS_ROLL;
  T1.wls_pitch = ESH_WLS_PITCH;
  T1.wls_yaw = ESH_WLS_YAW;
  T1.wls_lift = ESH_WLS_LIFT;
  T1.wls_thrust = ESH_WLS_THRUST;
  T1.wls_motors = ESH_WLS_MOTORS;
  T1.wls_meant = ESH_WLS_MEANT;
  T1.wls_tdiff = ESH_WLS_TDIFF;

  T1.wls_gamma_sq = ESH_GAMMA_SQ;

  T1.wls_min_mt = ESH_WLS_MIN_MT;

  // Inertia
  T1.I_XX = I_XX_0TILT;
  T1.I_YY = I_YY_0TILT; 
  T1.I_ZZ = I_ZZ_0TILT; 

  T1.aero_coeff = 0.0668; // 0.5 * density (1.22) * wing area (0.11)
  T1.chord = 0.15;
  T1.span = 0.73;
  T1.mR.dX = 0.09; // Lever arm (absolute value)
  T1.mR.dY = 0.12; // Lever arm (absolute value)
  T1.mL.dX = 0.09; // Lever arm (absolute value)
  T1.mL.dY = 0.12; // Lever arm (absolute value)
  T1.mB.dX = 0.25; // Lever arm (absolute value)
  T1.mB.dY = 0.0;  // Lever arm (absolute value)

  esh_counter = 0;
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
  T1.tiltr.rad  = 1.73329 * (actuator_state_filt_vect[ESH_CMD_MOTORMT] + actuator_state_filt_vect[ESH_CMD_MOTORTD]) / (MAX_PPRZ);
  Bound(T1.tiltr.rad, 0. , 1.73329)
  T1.tiltr.deg   = T1.tiltr.rad / M_PI * 180.;
  T1.tiltr.cosr  = cosf(T1.tiltr.rad);
  T1.tiltr.sinr  = sinf(T1.tiltr.rad);
  T1.tiltl.rad  = 1.73329 * (actuator_state_filt_vect[ESH_CMD_MOTORMT] - actuator_state_filt_vect[ESH_CMD_MOTORTD]) / (MAX_PPRZ);
  Bound(T1.tiltl.rad, 0. , 1.73329)
  T1.tiltl.deg   = T1.tiltl.rad / M_PI * 180.;
  T1.tiltl.cosr  = cosf(T1.tiltl.rad);
  T1.tiltl.sinr  = sinf(T1.tiltl.rad);
}

void eff_scheduling_heewing_update_thrust(void)
{//Thrust SP is normalized between 0. and 1.
  T1.mR.T = 3.5 * (actuator_state_filt_vect[ESH_CMD_MOTORR]) / (MAX_PPRZ);
  Bound(T1.mR.T, 0. , 3.5)
  T1.mL.T = 3.5 * (actuator_state_filt_vect[ESH_CMD_MOTORL]) / (MAX_PPRZ);
  Bound(T1.mL.T, 0. , 3.5)
  T1.mB.T = 3.5 * (actuator_state_filt_vect[ESH_CMD_MOTORB]) / (MAX_PPRZ);
  Bound(T1.mB.T, 0. , 3.5)

  //T1.mR.T = actuator_state_filt_vect[ESH_CMD_MOTORR];
  //T1.mL.T = actuator_state_filt_vect[ESH_CMD_MOTORL];
  //T1.mB.T = actuator_state_filt_vect[ESH_CMD_MOTORB];
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
  // Motor Right
  G1_T1[ESH_P][ESH_CMD_MOTORR]  = -0.666667 * (ROLL_EFF_MR * T1.tiltr.sinr * T1.mR.dY) / T1.I_XX;
  G1_T1[ESH_Q][ESH_CMD_MOTORR]  =  0.444444 * (PITCH_EFF_MR  * T1.tiltr.sinr * T1.mR.dX) / T1.I_YY;
  G1_T1[ESH_R][ESH_CMD_MOTORR]  = -0.444444 * (T1.yaw_eff_mr    * T1.tiltr.cosr * T1.mR.dY) / T1.I_ZZ;
  G1_T1[ESH_W][ESH_CMD_MOTORR]  = -0.567724 * (T1.lift_eff_mr * T1.tiltr.sinr) / MASS;
  G1_T1[ESH_U][ESH_CMD_MOTORR]  =  0.567724 * (T1.thrust_eff_mr * T1.tiltr.cosr) / MASS;
  G2_T1[ESH_CMD_MOTORR]         =  0; //T1.mR.dMdud / T1.I_ZZ;
  // Motor Left
  G1_T1[ESH_P][ESH_CMD_MOTORL]  =  0.666667 * (ROLL_EFF_ML * T1.tiltl.sinr * T1.mL.dY) / T1.I_XX;
  G1_T1[ESH_Q][ESH_CMD_MOTORL]  =  0.444444 * (PITCH_EFF_ML  * T1.tiltl.sinr * T1.mL.dX) / T1.I_YY;
  G1_T1[ESH_R][ESH_CMD_MOTORL]  =  0.444444 * (T1.yaw_eff_ml    * T1.tiltl.cosr * T1.mL.dY) / T1.I_ZZ;
  G1_T1[ESH_W][ESH_CMD_MOTORL]  = -0.567724 * (T1.lift_eff_ml * T1.tiltl.sinr) / MASS;
  G1_T1[ESH_U][ESH_CMD_MOTORL]  =  0.567724 * (T1.thrust_eff_ml * T1.tiltl.cosr) / MASS;
  G2_T1[ESH_CMD_MOTORL]         =  0; //T1.mL.dMdud / T1.I_ZZ;
  // Motor Back 
  G1_T1[ESH_Q][ESH_CMD_MOTORB]  = -0.444444 * (T1.pitch_eff_mb * T1.mB.dX) / T1.I_YY;
  G1_T1[ESH_W][ESH_CMD_MOTORB]  = -0.568035 * T1.lift_eff_mb / MASS;
  G2_T1[ESH_CMD_MOTORB]         =  0; //-T1.mB.dMdud / T1.I_ZZ;
  // Motor Mean Tilt
  G1_T1[ESH_Q][ESH_CMD_MOTORMT]  = 0.2 * (T1.mR.T * T1.tiltr.sinr * T1.mR.dX + T1.mL.T * T1.tiltl.sinr * T1.mL.dX) / T1.I_YY;
  G1_T1[ESH_W][ESH_CMD_MOTORMT]  = -T1.lift_eff_mt * 0.0003 * (T1.mR.T * T1.tiltr.cosr + T1.mL.T * T1.tiltl.cosr) / MASS;
  G1_T1[ESH_U][ESH_CMD_MOTORMT]  = -T1.thrust_eff_mt * 0.0003 * (T1.mR.T * T1.tiltr.sinr + T1.mL.T * T1.tiltl.sinr) / MASS;
  G2_T1[ESH_CMD_MOTORMT]         =  0; //T1.mR.dMdud / T1.I_ZZ;
  // Motor Tilt Diff
  G1_T1[ESH_P][ESH_CMD_MOTORTD]  = -1.536098 * (T1.mR.T * T1.tiltr.cosr * T1.mR.dY + T1.mL.T * T1.tiltl.cosr * T1.mL.dY) / (2 * T1.I_ZZ);
  G1_T1[ESH_R][ESH_CMD_MOTORTD]  = T1.yaw_eff_td * 1.536098 * (T1.mR.T * T1.tiltr.sinr * T1.mR.dY + T1.mL.T * T1.tiltl.sinr * T1.mL.dY) / (2 * T1.I_ZZ);
  G2_T1[ESH_CMD_MOTORTD]         =  0; //T1.mR.dMdud / T1.I_ZZ;
  // Criteria to not saturate control surfaces at low speed and start using them at around 5 m/s
  //if (T1.as < 5) {
  //  ctrl_surfaces_eff = 0; //-9.8 * T1.as + 50; // VERIFY IF IT WORKS AT VERY LOW SPEEDS (near 0)
  //} else {
  ctrl_surfaces_eff = 1;
  //}
  // Aileron
  G1_T1[ESH_P][ESH_CMD_AILERONS] = 1.0E-6 * T1.roll_eff_ail * ctrl_surfaces_eff * (T1.aero_coeff * T1.as2 * T1.span * ROLL_D2_AILERONS) / T1.I_XX;
 // Elevator
  G1_T1[ESH_Q][ESH_CMD_ELEVATOR] = 1.0E-6 * T1.pitch_eff_ele * ctrl_surfaces_eff * (T1.aero_coeff * T1.as2 * T1.chord * PITCH_D2_ELEVATOR) / T1.I_YY;
}

void sum_copy_EFF_MAT(void) {
  for (int8_t i = 0; i < INDI_OUTPUTS; i++) {
    for (int8_t j = 0; j < INDI_NUM_ACT; j++) {
        g1g2[i][j] = G1_T1[i][j] / INDI_G_SCALING;
    }
  }
}

void stabilization_indi_set_wls_settings(void)
{
  wls_stab_p.gamma_sq = T1.wls_gamma_sq;

  wls_stab_p.Wv[0] = T1.wls_roll;
  wls_stab_p.Wv[1] = T1.wls_pitch;
  wls_stab_p.Wv[2] = T1.wls_yaw;
  wls_stab_p.Wv[3] = T1.wls_lift;
  wls_stab_p.Wv[4] = T1.wls_thrust;

  wls_stab_p.Wu[0] = T1.wls_motors;
  wls_stab_p.Wu[1] = T1.wls_motors;
  wls_stab_p.Wu[2] = T1.wls_motors;
  wls_stab_p.Wu[3] = T1.wls_meant;
  wls_stab_p.Wu[4] = T1.wls_tdiff;
  wls_stab_p.Wu[5] = 1.0;
  wls_stab_p.Wu[6] = 1.0;

  for (int8_t i = 0; i < ESH_EFF_MAT_COLS_NB; i++) {
    switch (i) {
      case (ESH_CMD_MOTORR):
      case (ESH_CMD_MOTORL):
      case (ESH_CMD_MOTORB):
        wls_stab_p.u_min[i] = ESH_MOTOR_IDLE;
        wls_stab_p.u_max[i] = MAX_PPRZ;
        wls_stab_p.u_pref[i] = act_pref[i];
        break;
      case(ESH_CMD_MOTORTD):
        wls_stab_p.u_min[i] = -ESH_TILT_DIFF_MAX;
        wls_stab_p.u_max[i] = ESH_TILT_DIFF_MAX;
        wls_stab_p.u_pref[i] = act_pref[i];
        break;
      case(ESH_CMD_MOTORMT):
        wls_stab_p.u_min[i] = T1.wls_min_mt;
        wls_stab_p.u_max[i] = MAX_PPRZ - ESH_TILT_DIFF_MAX;
        wls_stab_p.u_pref[i] = act_pref[i];
        break;
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
float guidance_indi_get_liftd(float airspeed, float theta UNUSED) {
  return (-T1.aero_coeff * airspeed * airspeed * LIFT_D2_ALPHA);
}

float get_max_pusher_thrust(void) {
  return MAX_PPRZ * g1g2[ESH_U][ESH_CMD_MOTORR] + MAX_PPRZ * g1g2[ESH_U][ESH_CMD_MOTORL] + 
         MAX_PPRZ * g1g2[ESH_U][ESH_CMD_MOTORMT] + MAX_PPRZ * g1g2[ESH_U][ESH_CMD_MOTORTD];
}

void update_total_thrust(int32_t *cmd){
  cmd[COMMAND_THRUST]   = (actuator_state_filt_vect[ESH_CMD_MOTORR] + actuator_state_filt_vect[ESH_CMD_MOTORL] + actuator_state_filt_vect[ESH_CMD_MOTORB])/3;
}