/*
 * Copyright (C) 2025 Ramon Revilla Bouso <>
 *
 * This file is part of paparazzi.
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file firmwares/rotorcraft/guidance/guidance_indi_hybrid_tiltrotor.c
 *
 */

#include "firmwares/rotorcraft/guidance/guidance_indi_hybrid.h"
#include "firmwares/rotorcraft/guidance/guidance_indi_hybrid_tiltrotor.h"
#include "filters/low_pass_filter.h"
#include "state.h"
#include "generated/modules.h"

#ifndef GUIDANCE_INDI_ROLL_EFF_SCALING
#define GUIDANCE_INDI_ROLL_EFF_SCALING 1.0
#endif

#ifndef GUIDANCE_INDI_PITCH_EFF_SCALING
#define GUIDANCE_INDI_PITCH_EFF_SCALING 1.0
#endif

#ifndef GUIDANCE_INDI_LIFT_EFF_SCALING
#define GUIDANCE_INDI_LIFT_EFF_SCALING 1.0
#endif

#ifndef GUIDANCE_INDI_THRUST_EFF_SCALING
#define GUIDANCE_INDI_THRUST_EFF_SCALING 1.0
#endif

// WLS Guidance Roll Weight
#ifndef ESH_WLS_GUID_ROLL
#define ESH_WLS_GUID_ROLL 1000
#endif

// WLS Guidance Pitch Weight
#ifndef ESH_WLS_GUID_PITCH
#define ESH_WLS_GUID_PITCH 1000
#endif

// WLS Guidance FZ Weight
#ifndef ESH_WLS_GUID_FZ
#define ESH_WLS_GUID_FZ 200
#endif

// WLS Guidance FX Weight
#ifndef ESH_WLS_GUID_FX
#define ESH_WLS_GUID_FX 100
#endif

float gi_roll_eff_scaling = GUIDANCE_INDI_ROLL_EFF_SCALING;
float gi_pitch_eff_scaling = GUIDANCE_INDI_PITCH_EFF_SCALING;
float gi_lift_eff_scaling = GUIDANCE_INDI_LIFT_EFF_SCALING;
float gi_thrust_eff_scaling = GUIDANCE_INDI_THRUST_EFF_SCALING;

float bodyx_filter_cutoff = 0.2;

float wls_guid_roll = ESH_WLS_GUID_ROLL;
float wls_guid_pitch = ESH_WLS_GUID_PITCH;
float wls_guid_fz = ESH_WLS_GUID_FZ;
float wls_guid_fx = ESH_WLS_GUID_FX;

float bodyz_filter_cutoff = 0.2;

Butterworth2LowPass accel_bodyx_filt;
Butterworth2LowPass accel_bodyz_filt;

inline void guidance_indi_hybrid_set_wls_settings(float body_v[3], float roll_angle, float pitch_angle);

/**
 *
 * Call upon entering indi guidance
 */
void guidance_indi_tiltrotor_init(void) {
  float tau_bodyx = 1.0/(2.0*M_PI*bodyx_filter_cutoff);
  float tau_bodyz = 1.0/(2.0*M_PI*bodyz_filter_cutoff);
  float sample_time = 1.0 / PERIODIC_FREQUENCY;
  init_butterworth_2_low_pass(&accel_bodyx_filt, tau_bodyx, sample_time, 0.0);
  init_butterworth_2_low_pass(&accel_bodyz_filt, tau_bodyz, sample_time, 9.81);


}

/**
 * Low pass the accelerometer measurements to remove noise from vibrations.
 * The roll and pitch also need to be filtered to synchronize them with the
 * acceleration
 * Called as a periodic function with PERIODIC_FREQ
 */
void guidance_indi_tiltrotor_propagate_filters(void) {
   // Propagate filters
  float accelx = ACCEL_FLOAT_OF_BFP(stateGetAccelBody_i()->x);
  float accelz = ACCEL_FLOAT_OF_BFP(stateGetAccelBody_i()->z);
  update_butterworth_2_low_pass(&accel_bodyx_filt, accelx);
  update_butterworth_2_low_pass(&accel_bodyz_filt, accelz);
}

/**
 * Perform WLS
 *
 * @param Gmat Dynamics matrix
 * @param a_diff acceleration errors in earth frame
 * @param body_v 3D vector to write the control objective v
 */
void guidance_indi_calcg_wing(float Gmat[GUIDANCE_INDI_HYBRID_V][GUIDANCE_INDI_HYBRID_U], struct FloatVect3 a_diff, float body_v[GUIDANCE_INDI_HYBRID_V]) {
  // Get attitude
  struct FloatEulers eulers_zxy;
  float_eulers_of_quat_zxy(&eulers_zxy, stateGetNedToBodyQuat_f());

  /*Pre-calculate sines and cosines*/
  float sphi = sinf(eulers_zxy.phi);
  float cphi = cosf(eulers_zxy.phi);
  float stheta = sinf(eulers_zxy.theta);
  float ctheta = cosf(eulers_zxy.theta);
  float spsi = sinf(eulers_zxy.psi);
  float cpsi = cosf(eulers_zxy.psi);

  /*Force resultants*/
  float fx = MASS * accel_bodyx_filt.o[0];//thrust_estimated[0] * gi_roll_eff_scaling;
  float fz = MASS * accel_bodyz_filt.o[0];//thrust_estimated[2];

  // get the derivative of the lift wrt to theta
  float dfz = guidance_indi_get_liftd(stateGetAirspeed_f(), eulers_zxy.theta);

  // dPhi (Roll)
  Gmat[0][0] = fx * (-spsi * cphi * stheta) + fz * (spsi * cphi * ctheta) * gi_roll_eff_scaling;
  Gmat[1][0] = fx * (cpsi * cphi * stheta) + fz * (-cpsi * cphi * ctheta) * gi_roll_eff_scaling;
  Gmat[2][0] = fx * (sphi * stheta) + fz * (-sphi * ctheta) * gi_roll_eff_scaling;

  // dTheta (Pitch)
  Gmat[0][1] = fx * (-cpsi * stheta - spsi * sphi * ctheta) + fz * (cpsi * ctheta - spsi * sphi * stheta) * gi_pitch_eff_scaling + (cpsi * stheta + spsi * sphi * ctheta) * dfz;
  Gmat[1][1] = fx * (-spsi * stheta + cpsi * sphi * ctheta) + fz * (spsi * ctheta + cpsi * sphi * stheta) * gi_pitch_eff_scaling + (spsi * stheta - cpsi * sphi * ctheta) * dfz;
  Gmat[2][1] = fx * (-cphi * ctheta)                        + fz * (-cphi * stheta) * gi_pitch_eff_scaling                       + cphi * ctheta * dfz;

  // dfz
  Gmat[0][2] = (cpsi * stheta + spsi * sphi * ctheta) * gi_lift_eff_scaling;
  Gmat[1][2] = (spsi * stheta - cpsi * sphi * ctheta) * gi_lift_eff_scaling; 
  Gmat[2][2] = cphi * ctheta * gi_lift_eff_scaling;

  // dfx
  Gmat[0][3]  =  (cpsi * ctheta - spsi * sphi * stheta) * gi_thrust_eff_scaling; //0.2 for thrust scaling gives better results 
  Gmat[1][3]  =  (spsi * ctheta + cpsi * sphi * stheta) * gi_thrust_eff_scaling;
  Gmat[2][3]  = -stheta * cphi * gi_thrust_eff_scaling;

  body_v[GIHT_X] =  a_diff.x;
  body_v[GIHT_Y] =  a_diff.y;
  body_v[GIHT_Z] =  a_diff.z;
      
}

void guidance_indi_hybrid_set_wls_settings(float body_v[3], float roll_angle, float pitch_angle)
{
  // Weights evolution
  //float Wu_original[GUIDANCE_INDI_HYBRID_U] = GUIDANCE_INDI_WLS_WU;
  //float Wv_original[GUIDANCE_INDI_HYBRID_V] = GUIDANCE_INDI_WLS_PRIORITIES;

  //wls_guid_p.Wu[ESH_CMD_PITCH] = Wu_original[ESH_CMD_PITCH] + 5 * T1.as;
  //wls_guid_p.Wu[ESH_CMD_FX] = Wu_original[ESH_CMD_FX] + 300 * (1 - pow(M_E, (T1.as / 4.0)));
  //wls_guid_p.Wv[0] = Wv_original[0] + 3 * T1.as; //maybe could evolute with tilt angle

  wls_guid_p.Wu[ESH_CMD_ROLL] = wls_guid_roll;
  wls_guid_p.Wu[ESH_CMD_PITCH] = wls_guid_pitch;
  wls_guid_p.Wu[ESH_CMD_FZ] = wls_guid_fz;
  wls_guid_p.Wu[ESH_CMD_FX] = wls_guid_fx;
 
  // Set lower limits
  wls_guid_p.u_min[ESH_CMD_ROLL]  = -RadOfDeg(20) - roll_angle;
  wls_guid_p.u_min[ESH_CMD_PITCH] = -RadOfDeg(80) - pitch_angle;
  wls_guid_p.u_min[ESH_CMD_FZ]    = -20 - MASS * accel_bodyz_filt.o[0];
  wls_guid_p.u_min[ESH_CMD_FX]    = 0 - MASS * accel_bodyx_filt.o[0];

  // Set upper limits
  wls_guid_p.u_max[ESH_CMD_ROLL]  = RadOfDeg(20) - roll_angle;
  wls_guid_p.u_max[ESH_CMD_PITCH] = RadOfDeg(25) - pitch_angle;
  wls_guid_p.u_max[ESH_CMD_FZ]    = 0 - MASS * accel_bodyz_filt.o[0];
  wls_guid_p.u_max[ESH_CMD_FX]    = 20 - MASS * accel_bodyx_filt.o[0];

  // Set prefered states
  wls_guid_p.u_pref[ESH_CMD_ROLL]  = -roll_angle; // prefered delta roll angle
  wls_guid_p.u_pref[ESH_CMD_PITCH] = -pitch_angle; // + pitch_pref_rad;// prefered delta pitch angle

  wls_guid_p.u_pref[ESH_CMD_FZ]    =  MASS * accel_bodyz_filt.o[0];//thrust_estimated[2] // Low thrust better for efficiency
  wls_guid_p.u_pref[ESH_CMD_FX]    =  MASS * accel_bodyx_filt.o[0];//thrust_estimated[0] // solve the body acceleration
}